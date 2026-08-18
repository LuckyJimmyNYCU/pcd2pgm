// pcd2pgm - ROS 2 Jazzy port, file-output version
//
// Path convention:
//     <world_directory>/<input_subdir>/<map_name>.pcd     ->
//     <world_directory>/<output_subdir>/<map_name>/<output_name>.pgm
//     <world_directory>/<output_subdir>/<map_name>/<output_name>.yaml
//
// output_name defaults to map_name, so the usual result is
//     .../2Dmap/live_map_8f/live_map_8f.pgm
//
// In normal use only map_name changes. Full paths can still be forced with
// input_pcd and output_basepath.
//
// Pipeline:
//     load
//   -> optional statistical outlier removal (SOR)
//   -> optional coarse absolute-z crop
//   -> optional early voxel downsample
//   -> height filter, either
//        absolute:         keep z inside [thre_z_min, thre_z_max]
//        ground-relative:  estimate a local ground surface and keep
//                          (z - ground_z) inside [thre_z_min, thre_z_max]
//   -> optional voxel downsample
//   -> optional radius outlier removal
//   -> rasterise, write .pgm + .yaml that nav2_map_server can load directly
//
// No topic is published; the node writes the files and exits.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <ios>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <CSF.h>

namespace fs = std::filesystem;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// Nav2 map image conventions (same as nav2_map_server's map_saver output):
//   0   = occupied  (black)
//   254 = free      (white)
//   205 = unknown   (grey)
namespace pgm
{
constexpr uint8_t kOccupied = 0;
constexpr uint8_t kFree = 254;
constexpr uint8_t kUnknown = 205;
}  // namespace pgm

class Pcd2Pgm : public rclcpp::Node
{
public:
  Pcd2Pgm()
  : rclcpp::Node("pcd2pgm")
  {
    declareParameters();
    resolvePaths();

    // ---- load ---------------------------------------------------------
    auto cloud = std::make_shared<CloudT>();
    if (pcl::io::loadPCDFile<PointT>(input_pcd_.string(), *cloud) == -1) {
      throw std::runtime_error("Could not read PCD file: " + input_pcd_.string());
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu points from %s",
      cloud->points.size(), input_pcd_.string().c_str());
    logBounds("Input cloud", cloud);

    // Bounds for the output map are taken from this cloud when
    // bounds_from_full_cloud is true. SOR runs before it is captured so that
    // a handful of stray points cannot inflate the map by tens of metres.
    CloudT::Ptr work = cloud;

    // ---- statistical outlier removal ----------------------------------
    // Runs first: points floating below the floor would otherwise drag the
    // ground surface estimate down and open holes in the relative slice.
    if (use_statistical_filter_) {
      work = statisticalFilter(work);
      if (work->points.empty()) {
        throw std::runtime_error(
                "No points left after the statistical filter. Raise "
                "sor_std_ratio or lower sor_mean_k.");
      }
      logBounds("After SOR", work);
    }
    CloudT::Ptr bounds_cloud = work;

    // ---- coarse absolute-z crop ---------------------------------------
    // Optional. Trims sky, ceiling and gross z outliers before the ground
    // surface is estimated. Wide limits are fine; this is not the slice.
    if (use_coarse_z_filter_) {
      work = coarseZFilter(work);
      if (work->points.empty()) {
        throw std::runtime_error(
                "No points left after the coarse z filter. Widen "
                "coarse_z_min / coarse_z_max.");
      }
    }

    // ---- optional early downsample ------------------------------------
    // Only when voxel_before_height_filter is true. This sees the full z
    // range of the cloud, so the voxel grid can get very large; the height
    // filter first is usually the better order.
    if (use_voxel_filter_ && voxel_before_height_filter_) {
      work = voxelFilter(work);
      if (work->points.empty()) {
        throw std::runtime_error(
                "No points left after the voxel filter. Lower "
                "voxel_min_points_per_voxel or shrink voxel_leaf_*.");
      }
    }

    // ---- height filter ------------------------------------------------
    CloudT::Ptr passed = use_ground_relative_slice_
      ? groundRelativeSlice(work)
      : passThrough(work);
    if (passed->points.empty()) {
      throw std::runtime_error(
              use_ground_relative_slice_
              ? "No points left after the ground-relative slice. Check "
              "thre_z_min / thre_z_max; they are now measured from the "
              "local ground, so both are usually small positive numbers."
              : "No points left after the height filter. Check thre_z_min / "
              "thre_z_max against the actual z range of the cloud.");
    }

    // ---- downsample ---------------------------------------------------
    // Default position: the height band has already collapsed the z extent,
    // so the voxel grid stays small. The original cloud is still used for the
    // map bounds, so turning this on does not shift the origin or size.
    CloudT::Ptr downsampled = passed;
    if (use_voxel_filter_ && !voxel_before_height_filter_) {
      downsampled = voxelFilter(passed);
      if (downsampled->points.empty()) {
        throw std::runtime_error(
                "No points left after the voxel filter. Lower "
                "voxel_min_points_per_voxel or shrink voxel_leaf_*.");
      }
    }

    CloudT::Ptr obstacles = downsampled;
    if (use_radius_filter_) {
      obstacles = radiusFilter(downsampled);
      if (obstacles->points.empty()) {
        throw std::runtime_error(
                "No points left after the radius filter. Lower "
                "thres_point_count or raise thre_radius.");
      }
    }

    // ---- rasterise and write ------------------------------------------
    writeMap(bounds_cloud, obstacles);
  }

private:
  void declareParameters()
  {
    // The one setting that normally changes between runs. Used for both the
    // input file name and, unless output_name overrides it, the output name.
    map_name_ = declare_parameter<std::string>("map_name", "map");

    // Layout under the world package.
    world_directory_ = declare_parameter<std::string>("world_directory", "");
    input_subdir_ = declare_parameter<std::string>("input_subdir", "PCD");
    output_subdir_ = declare_parameter<std::string>("output_subdir", "2Dmap");

    // Escape hatches. Non-empty values win over the layout above.
    // input_pcd is a full path including .pcd.
    // output_basepath is a full path WITHOUT extension; .pgm and .yaml are
    // appended to it.
    input_pcd_override_ = declare_parameter<std::string>("input_pcd", "");
    output_basepath_override_ = declare_parameter<std::string>("output_basepath", "");
    // Empty means reuse map_name.
    output_name_ = declare_parameter<std::string>("output_name", "");

    // Statistical outlier removal. Drops points whose mean distance to their
    // sor_mean_k nearest neighbours is more than sor_std_ratio standard
    // deviations above the cloud average.
    use_statistical_filter_ = declare_parameter<bool>("use_statistical_filter", false);
    sor_mean_k_ = declare_parameter<int>("sor_mean_k", 30);
    sor_std_ratio_ = declare_parameter<double>("sor_std_ratio", 1.0);

    // Coarse absolute-z crop, applied before the ground surface estimate.
    use_coarse_z_filter_ = declare_parameter<bool>("use_coarse_z_filter", false);
    coarse_z_min_ = declare_parameter<double>("coarse_z_min", -5.0);
    coarse_z_max_ = declare_parameter<double>("coarse_z_max", 5.0);

    // Ground-relative slicing. When true, thre_z_min / thre_z_max are
    // measured from a locally estimated ground surface instead of from z = 0,
    // so a sloping or drifting floor still yields one consistent slice.
    use_ground_relative_slice_ =
      declare_parameter<bool>("use_ground_relative_slice", false);
    // "csf"  = cloth simulation filter, the method from Zhang et al. 2016
    // "grid" = per-cell low percentile of the raw cloud
    ground_method_ = declare_parameter<std::string>("ground_method", "csf");

    // CSF parameters. Same meaning as the reference implementation, so values
    // that work in the Python CSF module carry over unchanged.
    csf_slope_smooth_ = declare_parameter<bool>("csf_slope_smooth", true);
    csf_cloth_resolution_ = declare_parameter<double>("csf_cloth_resolution", 0.5);
    csf_rigidness_ = declare_parameter<int>("csf_rigidness", 2);
    csf_class_threshold_ = declare_parameter<double>("csf_class_threshold", 0.2);
    csf_max_iterations_ = declare_parameter<int>("csf_max_iterations", 500);
    csf_time_step_ = declare_parameter<double>("csf_time_step", 0.65);

    ground_cell_size_ = declare_parameter<double>("ground_cell_size", 1.0);
    ground_min_points_per_cell_ =
      declare_parameter<int>("ground_min_points_per_cell", 5);
    // 0.0 uses the lowest point in the cell; a small positive value is more
    // robust to single stray points below the floor.
    ground_percentile_ = declare_parameter<double>("ground_percentile", 0.05);
    // Cells more than this far from the median of their known neighbours are
    // discarded and refilled. 0 disables the check.
    ground_max_step_ = declare_parameter<double>("ground_max_step", 0.5);
    ground_smooth_passes_ = declare_parameter<int>("ground_smooth_passes", 2);
    ground_smooth_alpha_ = declare_parameter<double>("ground_smooth_alpha", 0.5);

    // Voxel downsampling. leaf_z can be larger than leaf_x / leaf_y: the map
    // is a 2D raster, so vertical detail inside the height band is not needed.
    use_voxel_filter_ = declare_parameter<bool>("use_voxel_filter", false);
    // false = downsample after the height filter (recommended). true = before,
    // which makes the voxel grid span the full z range of the cloud.
    voxel_before_height_filter_ =
      declare_parameter<bool>("voxel_before_height_filter", false);
    voxel_leaf_x_ = declare_parameter<double>("voxel_leaf_x", 0.05);
    voxel_leaf_y_ = declare_parameter<double>("voxel_leaf_y", 0.05);
    voxel_leaf_z_ = declare_parameter<double>("voxel_leaf_z", 0.05);
    // A voxel is kept only if it contains at least this many points, so
    // raising it also removes isolated noise. 1 = plain downsampling.
    voxel_min_points_per_voxel_ =
      declare_parameter<int>("voxel_min_points_per_voxel", 1);

    // Absolute when use_ground_relative_slice is false, relative to the local
    // ground when it is true.
    thre_z_min_ = declare_parameter<double>("thre_z_min", 0.2);
    thre_z_max_ = declare_parameter<double>("thre_z_max", 2.0);
    // false = keep points inside [min, max]; true = keep points outside.
    flag_pass_through_ = declare_parameter<bool>("flag_pass_through", false);

    use_radius_filter_ = declare_parameter<bool>("use_radius_filter", true);
    thre_radius_ = declare_parameter<double>("thre_radius", 0.5);
    thres_point_count_ = declare_parameter<int>("thres_point_count", 10);

    map_resolution_ = declare_parameter<double>("map_resolution", 0.05);
    padding_ = declare_parameter<double>("padding", 0.5);

    // When true, cells with no points become 205 (unknown) instead of 254
    // (free). Only useful when the costmap has track_unknown_space enabled.
    unknown_as_unknown_ = declare_parameter<bool>("unknown_as_unknown", false);

    // Bounds are taken from the full input cloud by default, so the map extent
    // matches other maps generated from the same PCD. Set false to crop to the
    // filtered obstacle points only.
    bounds_from_full_cloud_ = declare_parameter<bool>("bounds_from_full_cloud", true);

    save_intermediate_ = declare_parameter<bool>("save_intermediate", false);
  }

  void resolvePaths()
  {
    if (map_name_.empty()) {
      throw std::runtime_error("map_name must not be empty");
    }

    // ---- input --------------------------------------------------------
    if (!input_pcd_override_.empty()) {
      input_pcd_ = fs::path(input_pcd_override_);
    } else {
      if (world_directory_.empty()) {
        throw std::runtime_error(
                "Set world_directory (e.g. /work/nav2_ws/src/world), or give a "
                "full input_pcd path.");
      }
      input_pcd_ = fs::path(world_directory_) / input_subdir_ / (map_name_ + ".pcd");
    }

    if (!fs::exists(input_pcd_)) {
      throw std::runtime_error("Input PCD does not exist: " + input_pcd_.string());
    }

    // ---- output -------------------------------------------------------
    const std::string out_stem = output_name_.empty() ? map_name_ : output_name_;

    if (!output_basepath_override_.empty()) {
      output_base_ = fs::path(output_basepath_override_);
    } else {
      if (world_directory_.empty()) {
        throw std::runtime_error(
                "Set world_directory, or give a full output_basepath.");
      }
      // Each map gets its own folder:
      //   <world>/<output_subdir>/<map_name>/<out_stem>.pgm
      output_base_ =
        fs::path(world_directory_) / output_subdir_ / map_name_ / out_stem;
    }

    // Strip a trailing .pgm / .yaml if one was given by mistake, so that
    // output_basepath:=.../foo.pgm does not produce foo.pgm.pgm.
    const std::string ext = output_base_.extension().string();
    if (ext == ".pgm" || ext == ".yaml" || ext == ".yml") {
      output_base_.replace_extension();
    }

    const fs::path out_dir = output_base_.parent_path();
    if (!out_dir.empty()) {
      std::error_code ec;
      fs::create_directories(out_dir, ec);
      if (ec && !fs::exists(out_dir)) {
        throw std::runtime_error(
                "Cannot create output directory " + out_dir.string() + ": " +
                ec.message());
      }
    }

    RCLCPP_INFO(get_logger(), "map_name: %s", map_name_.c_str());
    RCLCPP_INFO(get_logger(), "Input:    %s", input_pcd_.string().c_str());
    RCLCPP_INFO(get_logger(), "Output:   %s.pgm / .yaml", output_base_.string().c_str());
    if (use_ground_relative_slice_ &&
      ground_method_ != "csf" && ground_method_ != "grid")
    {
      throw std::runtime_error(
              "ground_method must be \"csf\" or \"grid\", got \"" +
              ground_method_ + "\"");
    }

    const std::string mode = use_ground_relative_slice_
      ? "ground-relative (" + ground_method_ + ")"
      : std::string("absolute z");
    RCLCPP_INFO(get_logger(), "Height filter mode: %s", mode.c_str());
  }

  // Prints the XYZ extent of a cloud. Worth looking at when the voxel grid
  // complains: a span far larger than the building means stray far-away
  // points, which also inflate the output map because the bounds come from
  // the full cloud.
  void logBounds(const char * label, const CloudT::Ptr & cloud)
  {
    if (cloud->points.empty()) {
      return;
    }
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    RCLCPP_INFO(get_logger(),
      "%s extent: x [%.2f, %.2f] %.2f m, y [%.2f, %.2f] %.2f m, "
      "z [%.2f, %.2f] %.2f m",
      label,
      min_pt[0], max_pt[0], max_pt[0] - min_pt[0],
      min_pt[1], max_pt[1], max_pt[1] - min_pt[1],
      min_pt[2], max_pt[2], max_pt[2] - min_pt[2]);
  }

  // ---------------------------------------------------------------------
  // Filters
  // ---------------------------------------------------------------------

  CloudT::Ptr statisticalFilter(const CloudT::Ptr & input)
  {
    if (sor_mean_k_ < 1) {
      throw std::runtime_error("sor_mean_k must be at least 1");
    }
    if (static_cast<int>(input->points.size()) <= sor_mean_k_) {
      throw std::runtime_error(
              "Cloud has fewer points than sor_mean_k; lower sor_mean_k.");
    }

    auto out = std::make_shared<CloudT>();

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(input);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_std_ratio_);
    sor.filter(*out);

    RCLCPP_INFO(get_logger(),
      "After statistical filter (mean_k=%d, std_ratio=%.3f): %zu points "
      "(from %zu)",
      sor_mean_k_, sor_std_ratio_, out->points.size(), input->points.size());

    if (save_intermediate_ && !out->points.empty()) {
      pcl::io::savePCDFileBinary(output_base_.string() + "_sor.pcd", *out);
    }
    return out;
  }

  CloudT::Ptr coarseZFilter(const CloudT::Ptr & input)
  {
    if (coarse_z_min_ >= coarse_z_max_) {
      throw std::runtime_error("coarse_z_min must be less than coarse_z_max");
    }

    auto out = std::make_shared<CloudT>();

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
      static_cast<float>(coarse_z_min_), static_cast<float>(coarse_z_max_));
    pass.filter(*out);

    RCLCPP_INFO(get_logger(), "After coarse z crop [%.3f, %.3f]: %zu points",
      coarse_z_min_, coarse_z_max_, out->points.size());
    return out;
  }

  CloudT::Ptr voxelFilter(const CloudT::Ptr & input)
  {
    if (voxel_leaf_x_ <= 0.0 || voxel_leaf_y_ <= 0.0 || voxel_leaf_z_ <= 0.0) {
      throw std::runtime_error("voxel_leaf_x / _y / _z must be greater than zero");
    }

    // PCL addresses voxels with a single int32 index. If the bounding box
    // needs more voxels than that, VoxelGrid only prints a warning and hands
    // the cloud back unfiltered, which is easy to miss in the log. Check it
    // here and fail with a message that says what to do.
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::getMinMax3D(*input, min_pt, max_pt);

    const double nx = std::floor((max_pt[0] - min_pt[0]) / voxel_leaf_x_) + 1.0;
    const double ny = std::floor((max_pt[1] - min_pt[1]) / voxel_leaf_y_) + 1.0;
    const double nz = std::floor((max_pt[2] - min_pt[2]) / voxel_leaf_z_) + 1.0;
    const double voxel_count = nx * ny * nz;

    if (voxel_count > static_cast<double>(std::numeric_limits<int32_t>::max())) {
      RCLCPP_ERROR(get_logger(),
        "Voxel grid would be %.0f x %.0f x %.0f = %.0f cells over "
        "x %.2f m, y %.2f m, z %.2f m",
        nx, ny, nz, voxel_count,
        max_pt[0] - min_pt[0], max_pt[1] - min_pt[1], max_pt[2] - min_pt[2]);
      throw std::runtime_error(
              "voxel grid exceeds PCL's int32 cell index. Fixes, in order of "
              "preference: (1) set voxel_before_height_filter to false so the "
              "height band shrinks the z extent first; (2) remove far-away "
              "stray points from the PCD, which also shrink the output map; "
              "(3) increase voxel_leaf_x / _y / _z.");
    }

    auto out = std::make_shared<CloudT>();

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(
      static_cast<float>(voxel_leaf_x_),
      static_cast<float>(voxel_leaf_y_),
      static_cast<float>(voxel_leaf_z_));
    vg.setMinimumPointsNumberPerVoxel(
      static_cast<unsigned int>(std::max(1, voxel_min_points_per_voxel_)));
    vg.filter(*out);

    RCLCPP_INFO(get_logger(),
      "After voxel filter (leaf=%.3f/%.3f/%.3f, min_points=%d): "
      "%zu points (from %zu)",
      voxel_leaf_x_, voxel_leaf_y_, voxel_leaf_z_, voxel_min_points_per_voxel_,
      out->points.size(), input->points.size());

    if (save_intermediate_ && !out->points.empty()) {
      pcl::io::savePCDFileBinary(output_base_.string() + "_voxel.pcd", *out);
    }
    return out;
  }

  CloudT::Ptr passThrough(const CloudT::Ptr & input)
  {
    auto out = std::make_shared<CloudT>();

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
      static_cast<float>(thre_z_min_), static_cast<float>(thre_z_max_));
    pass.setNegative(flag_pass_through_);
    pass.filter(*out);

    RCLCPP_INFO(get_logger(), "After height filter [%.3f, %.3f]: %zu points",
      thre_z_min_, thre_z_max_, out->points.size());

    if (save_intermediate_ && !out->points.empty()) {
      pcl::io::savePCDFileBinary(output_base_.string() + "_pass.pcd", *out);
    }
    return out;
  }

  CloudT::Ptr radiusFilter(const CloudT::Ptr & input)
  {
    auto out = std::make_shared<CloudT>();

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(input);
    ror.setRadiusSearch(thre_radius_);
    ror.setMinNeighborsInRadius(thres_point_count_);
    ror.filter(*out);

    RCLCPP_INFO(get_logger(),
      "After radius filter (r=%.3f, min_neighbors=%d): %zu points",
      thre_radius_, thres_point_count_, out->points.size());

    if (save_intermediate_ && !out->points.empty()) {
      pcl::io::savePCDFileBinary(output_base_.string() + "_radius.pcd", *out);
    }
    return out;
  }

  // ---------------------------------------------------------------------
  // Ground surface estimation and relative slicing
  // ---------------------------------------------------------------------

  enum class CellStat
  {
    kPercentile,   // low percentile of every point in the cell (grid method)
    kMean          // mean of the classified ground points (csf method)
  };

  // Runs CSF and returns the subset of the cloud it classified as ground.
  //
  // CSF inverts the cloud in z and drops a simulated cloth onto it, so the
  // cloth comes to rest on the ground rather than on canopy or rooftops.
  // Points within csf_class_threshold of the settled cloth are ground.
  CloudT::Ptr csfGroundPoints(const CloudT::Ptr & input)
  {
    if (csf_cloth_resolution_ <= 0.0) {
      throw std::runtime_error("csf_cloth_resolution must be greater than zero");
    }
    if (csf_class_threshold_ <= 0.0) {
      throw std::runtime_error("csf_class_threshold must be greater than zero");
    }

    std::vector<csf::Point> pts;
    pts.reserve(input->points.size());
    // Index back into the PCL cloud, since non-finite points are skipped and
    // CSF returns positions in its own compacted vector.
    std::vector<size_t> source_index;
    source_index.reserve(input->points.size());

    for (size_t i = 0; i < input->points.size(); ++i) {
      const auto & p = input->points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      pts.emplace_back(
        static_cast<double>(p.x), static_cast<double>(p.y),
        static_cast<double>(p.z));
      source_index.push_back(i);
    }

    if (pts.size() < 16) {
      throw std::runtime_error("Too few finite points to run CSF");
    }

    RCLCPP_INFO(get_logger(),
      "Running CSF on %zu points (cloth_resolution=%.3f, rigidness=%d, "
      "class_threshold=%.3f, iterations=%d, time_step=%.3f, slope_smooth=%s)",
      pts.size(), csf_cloth_resolution_, csf_rigidness_, csf_class_threshold_,
      csf_max_iterations_, csf_time_step_, csf_slope_smooth_ ? "true" : "false");

    CSF csf;
    csf.params.bSloopSmooth = csf_slope_smooth_;
    csf.params.cloth_resolution = csf_cloth_resolution_;
    csf.params.rigidness = std::clamp(csf_rigidness_, 1, 3);
    csf.params.class_threshold = csf_class_threshold_;
    csf.params.interations = std::max(1, csf_max_iterations_);
    csf.params.time_step = csf_time_step_;
    csf.setPointCloud(pts);

    std::vector<int> ground_idx;
    std::vector<int> offground_idx;
    // The third argument must stay false: true makes CSF dump cloth_nodes.txt
    // into the current working directory.
    csf.do_filtering(ground_idx, offground_idx, false);

    auto out = std::make_shared<CloudT>();
    out->points.reserve(ground_idx.size());
    for (int idx : ground_idx) {
      if (idx < 0 || static_cast<size_t>(idx) >= source_index.size()) {
        continue;
      }
      out->points.push_back(input->points[source_index[static_cast<size_t>(idx)]]);
    }
    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;

    RCLCPP_INFO(get_logger(),
      "CSF ground: %zu points, non-ground: %zu points (%.1f%% ground)",
      out->points.size(), offground_idx.size(),
      100.0 * static_cast<double>(out->points.size()) /
      static_cast<double>(pts.size()));

    if (out->points.empty()) {
      throw std::runtime_error(
              "CSF classified no points as ground. Reduce csf_cloth_resolution, "
              "or raise csf_class_threshold.");
    }

    if (save_intermediate_) {
      pcl::io::savePCDFileBinary(output_base_.string() + "_csf_ground.pcd", *out);
    }
    return out;
  }

  // A regular XY grid holding one ground height per cell. Every cell is
  // finite after buildGroundGrid() returns: unobserved cells are filled from
  // their nearest known neighbour.
  struct GroundGrid
  {
    std::vector<double> z;
    int width{0};
    int height{0};
    double cell{1.0};
    double x_min{0.0};
    double y_min{0.0};
  };

  // extent_cloud fixes the grid bounds, so the surface always spans every
  // point that will later be sliced. value_cloud supplies the heights: the
  // whole cloud for the grid method, the CSF ground subset for the csf one.
  GroundGrid buildGroundGrid(
    const CloudT::Ptr & extent_cloud, const CloudT::Ptr & value_cloud,
    CellStat stat)
  {
    if (ground_cell_size_ <= 0.0) {
      throw std::runtime_error("ground_cell_size must be greater than zero");
    }
    if (ground_percentile_ < 0.0 || ground_percentile_ > 1.0) {
      throw std::runtime_error("ground_percentile must be between 0.0 and 1.0");
    }

    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();
    bool any = false;

    for (const auto & p : extent_cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        continue;
      }
      x_min = std::min(x_min, static_cast<double>(p.x));
      x_max = std::max(x_max, static_cast<double>(p.x));
      y_min = std::min(y_min, static_cast<double>(p.y));
      y_max = std::max(y_max, static_cast<double>(p.y));
      any = true;
    }
    if (!any) {
      throw std::runtime_error("Cloud has no finite points for ground estimation");
    }

    GroundGrid g;
    g.cell = ground_cell_size_;
    g.x_min = x_min;
    g.y_min = y_min;
    g.width = std::max(2, static_cast<int>(std::floor((x_max - x_min) / g.cell)) + 2);
    g.height = std::max(2, static_cast<int>(std::floor((y_max - y_min) / g.cell)) + 2);

    const double cell_count =
      static_cast<double>(g.width) * static_cast<double>(g.height);
    if (cell_count > 5.0e7) {
      throw std::runtime_error(
              "Ground grid would need more than 5e7 cells. Increase "
              "ground_cell_size, or trim far-away stray points from the PCD.");
    }

    const size_t n_cells = static_cast<size_t>(g.width) * static_cast<size_t>(g.height);

    // Per-cell z samples. Total storage is one double per contributing point,
    // which is the same order as the cloud itself.
    std::vector<std::vector<double>> samples(n_cells);
    for (const auto & p : value_cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const int ix = std::clamp(
        static_cast<int>(std::floor((p.x - g.x_min) / g.cell)), 0, g.width - 1);
      const int iy = std::clamp(
        static_cast<int>(std::floor((p.y - g.y_min) / g.cell)), 0, g.height - 1);
      samples[static_cast<size_t>(iy) * static_cast<size_t>(g.width) +
        static_cast<size_t>(ix)].push_back(static_cast<double>(p.z));
    }

    const int min_pts = std::max(1, ground_min_points_per_cell_);

    g.z.assign(n_cells, std::numeric_limits<double>::quiet_NaN());
    std::vector<uint8_t> known(n_cells, 0);
    size_t known_count = 0;

    for (size_t i = 0; i < n_cells; ++i) {
      auto & v = samples[i];
      if (static_cast<int>(v.size()) < min_pts) {
        continue;
      }
      if (stat == CellStat::kMean) {
        double sum = 0.0;
        for (double z : v) {
          sum += z;
        }
        g.z[i] = sum / static_cast<double>(v.size());
      } else {
        // Low percentile of the cell rather than the strict minimum, so a
        // single point below the floor cannot define the ground there.
        size_t k = static_cast<size_t>(
          std::llround(ground_percentile_ * static_cast<double>(v.size() - 1)));
        k = std::min(k, v.size() - 1);
        std::nth_element(v.begin(), v.begin() + static_cast<long>(k), v.end());
        g.z[i] = v[k];
      }
      known[i] = 1;
      ++known_count;
    }
    samples.clear();
    samples.shrink_to_fit();

    const double empty_fraction =
      1.0 - static_cast<double>(known_count) / static_cast<double>(n_cells);

    RCLCPP_INFO(get_logger(),
      "Ground grid: %d x %d cells @ %.2f m, known %zu / %zu (%.1f%% empty)",
      g.width, g.height, g.cell, known_count, n_cells, 100.0 * empty_fraction);

    if (known_count == 0) {
      throw std::runtime_error(
              "No ground cells were generated. Lower ground_min_points_per_cell "
              "or increase ground_cell_size.");
    }

    // A large empty fraction after CSF almost always means the cloth failed to
    // reach part of the terrain, which shows up as whole regions with no
    // ground points at all.
    if (ground_method_ == "csf" && empty_fraction > 0.30) {
      RCLCPP_WARN(get_logger(),
        "%.0f%% of the ground grid has no CSF ground points. The cloth "
        "probably could not follow the terrain. Halve csf_cloth_resolution "
        "and re-run; see the note on slope x cloth_resolution in the config.",
        100.0 * empty_fraction);
    }

    rejectGroundSteps(g, known, known_count);
    fillNearest(g, known);
    smoothGrid(g);
    warnOnSteepness(g);
    return g;
  }

  // Discards cells that sit far above or below the median of their known
  // neighbours. Catches cells whose only returns are a roof, a parked vehicle
  // or a pallet, which would otherwise lift the local ground and hide real
  // obstacles standing on it.
  void rejectGroundSteps(
    GroundGrid & g, std::vector<uint8_t> & known, size_t & known_count)
  {
    if (ground_max_step_ <= 0.0) {
      return;
    }

    const std::vector<double> snapshot_z = g.z;
    const std::vector<uint8_t> snapshot_known = known;
    size_t rejected = 0;

    for (int y = 0; y < g.height; ++y) {
      for (int x = 0; x < g.width; ++x) {
        const size_t idx =
          static_cast<size_t>(y) * static_cast<size_t>(g.width) +
          static_cast<size_t>(x);
        if (!snapshot_known[idx]) {
          continue;
        }

        std::vector<double> nb;
        nb.reserve(8);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= g.width || ny < 0 || ny >= g.height) {
              continue;
            }
            const size_t n_idx =
              static_cast<size_t>(ny) * static_cast<size_t>(g.width) +
              static_cast<size_t>(nx);
            if (snapshot_known[n_idx]) {
              nb.push_back(snapshot_z[n_idx]);
            }
          }
        }
        // Too few neighbours to judge; leave the cell alone.
        if (nb.size() < 4) {
          continue;
        }

        const size_t mid = nb.size() / 2;
        std::nth_element(nb.begin(), nb.begin() + static_cast<long>(mid), nb.end());
        const double median = nb[mid];

        if (std::fabs(snapshot_z[idx] - median) > ground_max_step_) {
          known[idx] = 0;
          g.z[idx] = std::numeric_limits<double>::quiet_NaN();
          ++rejected;
        }
      }
    }

    known_count -= rejected;
    RCLCPP_INFO(get_logger(),
      "Ground step rejection (max_step=%.3f m): dropped %zu cells, %zu remain",
      ground_max_step_, rejected, known_count);

    if (known_count == 0) {
      throw std::runtime_error(
              "Ground step rejection removed every cell. Raise ground_max_step "
              "or set it to 0.0 to disable the check.");
    }
  }

  // Breadth-first flood fill from the known cells, so every unobserved cell
  // takes the value of its nearest known neighbour in grid distance.
  static void fillNearest(GroundGrid & g, const std::vector<uint8_t> & known)
  {
    std::vector<uint8_t> seen = known;
    std::deque<std::pair<int, int>> q;

    for (int y = 0; y < g.height; ++y) {
      for (int x = 0; x < g.width; ++x) {
        const size_t idx =
          static_cast<size_t>(y) * static_cast<size_t>(g.width) +
          static_cast<size_t>(x);
        if (seen[idx]) {
          q.emplace_back(y, x);
        }
      }
    }

    while (!q.empty()) {
      const auto [y, x] = q.front();
      q.pop_front();
      const size_t idx =
        static_cast<size_t>(y) * static_cast<size_t>(g.width) +
        static_cast<size_t>(x);
      const double v = g.z[idx];

      const int dys[4] = {-1, 1, 0, 0};
      const int dxs[4] = {0, 0, -1, 1};
      for (int k = 0; k < 4; ++k) {
        const int ny = y + dys[k];
        const int nx = x + dxs[k];
        if (nx < 0 || nx >= g.width || ny < 0 || ny >= g.height) {
          continue;
        }
        const size_t n_idx =
          static_cast<size_t>(ny) * static_cast<size_t>(g.width) +
          static_cast<size_t>(nx);
        if (seen[n_idx]) {
          continue;
        }
        seen[n_idx] = 1;
        g.z[n_idx] = v;
        q.emplace_back(ny, nx);
      }
    }
  }

  // 3x3 box blur with edge clamping, blended by alpha. Removes the blocky
  // steps the per-cell statistic leaves behind, so the relative slice does
  // not develop seams along cell borders.
  void smoothGrid(GroundGrid & g) const
  {
    const int passes = std::max(0, ground_smooth_passes_);
    const double alpha = std::clamp(ground_smooth_alpha_, 0.0, 1.0);
    if (passes == 0 || alpha == 0.0) {
      return;
    }

    std::vector<double> next(g.z.size());
    for (int p = 0; p < passes; ++p) {
      for (int y = 0; y < g.height; ++y) {
        for (int x = 0; x < g.width; ++x) {
          double sum = 0.0;
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
              const int ny = std::clamp(y + dy, 0, g.height - 1);
              const int nx = std::clamp(x + dx, 0, g.width - 1);
              sum += g.z[
                static_cast<size_t>(ny) * static_cast<size_t>(g.width) +
                static_cast<size_t>(nx)];
            }
          }
          const size_t idx =
            static_cast<size_t>(y) * static_cast<size_t>(g.width) +
            static_cast<size_t>(x);
          next[idx] = (1.0 - alpha) * g.z[idx] + alpha * (sum / 9.0);
        }
      }
      g.z.swap(next);
    }
  }

  // CSF's cloth can only follow terrain while the height change between two
  // adjacent cloth particles stays small; past roughly 0.3 m per particle
  // spacing the cloth bridges the slope instead of draping onto it and whole
  // regions come back unclassified. The recovered surface tells us the slope,
  // so the check can be made after the fact.
  void warnOnSteepness(const GroundGrid & g) const
  {
    if (ground_method_ != "csf") {
      return;
    }

    std::vector<double> diffs;
    diffs.reserve(g.z.size());
    for (int y = 0; y < g.height; ++y) {
      for (int x = 0; x + 1 < g.width; ++x) {
        const size_t i =
          static_cast<size_t>(y) * static_cast<size_t>(g.width) +
          static_cast<size_t>(x);
        diffs.push_back(std::fabs(g.z[i + 1] - g.z[i]));
      }
    }
    if (diffs.size() < 20) {
      return;
    }

    const size_t k = static_cast<size_t>(0.95 * static_cast<double>(diffs.size() - 1));
    std::nth_element(diffs.begin(), diffs.begin() + static_cast<long>(k), diffs.end());
    const double slope = diffs[k] / g.cell;          // metres per metre
    const double step = slope * csf_cloth_resolution_;

    RCLCPP_INFO(get_logger(),
      "Ground slope (95th percentile): %.3f m/m = %.1f deg; "
      "height step per cloth particle: %.3f m",
      slope, std::atan(slope) * 180.0 / M_PI, step);

    if (step > 0.30) {
      // The slope is measured from the surface CSF produced. If the cloth
      // already failed, that surface is flatter than the real terrain, so
      // this suggestion is an upper bound rather than a safe value.
      RCLCPP_WARN(get_logger(),
        "Height step per cloth particle is %.3f m, above the ~0.3 m that CSF "
        "can drape over. Try csf_cloth_resolution %.2f m or less, and keep "
        "halving it while this warning persists.",
        step, 0.30 / std::max(slope, 1e-6));
    }
  }

  // Bilinear sample of the ground surface at an arbitrary XY position.
  static double sampleGround(const GroundGrid & g, double px, double py)
  {
    const double fx = (px - g.x_min) / g.cell;
    const double fy = (py - g.y_min) / g.cell;
    const int x0 = std::clamp(static_cast<int>(std::floor(fx)), 0, g.width - 2);
    const int y0 = std::clamp(static_cast<int>(std::floor(fy)), 0, g.height - 2);
    const double tx = std::clamp(fx - static_cast<double>(x0), 0.0, 1.0);
    const double ty = std::clamp(fy - static_cast<double>(y0), 0.0, 1.0);

    const size_t w = static_cast<size_t>(g.width);
    const size_t i00 = static_cast<size_t>(y0) * w + static_cast<size_t>(x0);
    const size_t i10 = i00 + 1;
    const size_t i01 = i00 + w;
    const size_t i11 = i01 + 1;

    return (1.0 - tx) * (1.0 - ty) * g.z[i00] +
           tx * (1.0 - ty) * g.z[i10] +
           (1.0 - tx) * ty * g.z[i01] +
           tx * ty * g.z[i11];
  }

  CloudT::Ptr groundRelativeSlice(const CloudT::Ptr & input)
  {
    if (thre_z_min_ >= thre_z_max_) {
      throw std::runtime_error("thre_z_min must be less than thre_z_max");
    }

    GroundGrid g;
    if (ground_method_ == "csf") {
      const CloudT::Ptr ground = csfGroundPoints(input);
      g = buildGroundGrid(input, ground, CellStat::kMean);
    } else if (ground_method_ == "grid") {
      g = buildGroundGrid(input, input, CellStat::kPercentile);
    } else {
      throw std::runtime_error(
              "ground_method must be \"csf\" or \"grid\", got \"" +
              ground_method_ + "\"");
    }

    if (save_intermediate_) {
      writeGroundSurface(g);
    }

    auto out = std::make_shared<CloudT>();
    out->points.reserve(input->points.size() / 4 + 1);

    double rel_min = std::numeric_limits<double>::max();
    double rel_max = std::numeric_limits<double>::lowest();

    for (const auto & p : input->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const double ground_z = sampleGround(g, p.x, p.y);
      const double rel = static_cast<double>(p.z) - ground_z;
      rel_min = std::min(rel_min, rel);
      rel_max = std::max(rel_max, rel);

      const bool inside = (rel >= thre_z_min_) && (rel <= thre_z_max_);
      if (inside == flag_pass_through_) {
        continue;
      }
      out->points.push_back(p);
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;

    RCLCPP_INFO(get_logger(),
      "Relative height range in cloud: [%.3f, %.3f] m", rel_min, rel_max);
    RCLCPP_INFO(get_logger(),
      "After ground-relative slice [%.3f, %.3f]: %zu points (from %zu)",
      thre_z_min_, thre_z_max_, out->points.size(), input->points.size());

    if (save_intermediate_ && !out->points.empty()) {
      pcl::io::savePCDFileBinary(output_base_.string() + "_pass.pcd", *out);
    }
    return out;
  }

  // Dumps the estimated ground as a point per cell, so it can be overlaid on
  // the input cloud in a viewer to check the surface before trusting a slice.
  void writeGroundSurface(const GroundGrid & g) const
  {
    auto surface = std::make_shared<CloudT>();
    surface->points.reserve(g.z.size());
    for (int y = 0; y < g.height; ++y) {
      for (int x = 0; x < g.width; ++x) {
        PointT p;
        p.x = static_cast<float>(g.x_min + (static_cast<double>(x) + 0.5) * g.cell);
        p.y = static_cast<float>(g.y_min + (static_cast<double>(y) + 0.5) * g.cell);
        p.z = static_cast<float>(
          g.z[static_cast<size_t>(y) * static_cast<size_t>(g.width) +
          static_cast<size_t>(x)]);
        surface->points.push_back(p);
      }
    }
    surface->width = static_cast<uint32_t>(surface->points.size());
    surface->height = 1;
    surface->is_dense = true;
    pcl::io::savePCDFileBinary(
      output_base_.string() + "_ground_surface.pcd", *surface);
  }

  // ---------------------------------------------------------------------
  // Rasterisation and output
  // ---------------------------------------------------------------------

  struct Bounds
  {
    double x_min{std::numeric_limits<double>::max()};
    double x_max{std::numeric_limits<double>::lowest()};
    double y_min{std::numeric_limits<double>::max()};
    double y_max{std::numeric_limits<double>::lowest()};
    bool valid{false};
  };

  static Bounds computeBounds(const CloudT::Ptr & cloud)
  {
    Bounds b;
    for (const auto & p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        continue;
      }
      b.x_min = std::min(b.x_min, static_cast<double>(p.x));
      b.x_max = std::max(b.x_max, static_cast<double>(p.x));
      b.y_min = std::min(b.y_min, static_cast<double>(p.y));
      b.y_max = std::max(b.y_max, static_cast<double>(p.y));
      b.valid = true;
    }
    return b;
  }

  void writeMap(const CloudT::Ptr & full_cloud, const CloudT::Ptr & obstacles)
  {
    Bounds b = computeBounds(bounds_from_full_cloud_ ? full_cloud : obstacles);
    if (!b.valid) {
      throw std::runtime_error("Cloud has no finite XY points");
    }

    b.x_min -= padding_;
    b.y_min -= padding_;
    b.x_max += padding_;
    b.y_max += padding_;

    const uint32_t width =
      static_cast<uint32_t>(std::ceil((b.x_max - b.x_min) / map_resolution_)) + 1u;
    const uint32_t height =
      static_cast<uint32_t>(std::ceil((b.y_max - b.y_min) / map_resolution_)) + 1u;

    const uint8_t background = unknown_as_unknown_ ? pgm::kUnknown : pgm::kFree;
    std::vector<uint8_t> image(
      static_cast<size_t>(width) * static_cast<size_t>(height), background);

    size_t marked = 0;
    for (const auto & p : obstacles->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        continue;
      }
      const int gx = static_cast<int>(std::floor((p.x - b.x_min) / map_resolution_));
      const int gy = static_cast<int>(std::floor((p.y - b.y_min) / map_resolution_));

      if (gx < 0 || gx >= static_cast<int>(width)) {
        continue;
      }
      if (gy < 0 || gy >= static_cast<int>(height)) {
        continue;
      }

      // PGM rows run top-to-bottom; the map's y axis runs bottom-to-top.
      const size_t row = static_cast<size_t>(height - 1u - static_cast<uint32_t>(gy));
      image[row * static_cast<size_t>(width) + static_cast<size_t>(gx)] = pgm::kOccupied;
      ++marked;
    }

    const std::string pgm_path = output_base_.string() + ".pgm";
    const std::string yaml_path = output_base_.string() + ".yaml";
    const std::string pgm_basename = output_base_.filename().string() + ".pgm";

    writePgm(pgm_path, image, width, height);
    writeYaml(yaml_path, pgm_basename, b.x_min, b.y_min);

    RCLCPP_INFO(get_logger(), "Map size:      %u x %u cells", width, height);
    RCLCPP_INFO(get_logger(), "Physical size: %.2f x %.2f m",
      width * map_resolution_, height * map_resolution_);
    RCLCPP_INFO(get_logger(), "Origin:        [%.3f, %.3f, 0.0]", b.x_min, b.y_min);
    RCLCPP_INFO(get_logger(), "Occupied:      %zu / %zu cells", marked, image.size());
    RCLCPP_INFO(get_logger(), "PGM:  %s", pgm_path.c_str());
    RCLCPP_INFO(get_logger(), "YAML: %s", yaml_path.c_str());
  }

  static void writePgm(
    const std::string & path, const std::vector<uint8_t> & image,
    uint32_t width, uint32_t height)
  {
    std::ofstream f(path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("Cannot open for writing: " + path);
    }
    f << "P5\n";
    f << "# generated by pcd2pgm\n";
    f << width << " " << height << "\n";
    f << "255\n";
    f.write(reinterpret_cast<const char *>(image.data()),
      static_cast<std::streamsize>(image.size()));
    if (!f) {
      throw std::runtime_error("Failed while writing: " + path);
    }
  }

  void writeYaml(
    const std::string & path, const std::string & image_name,
    double origin_x, double origin_y) const
  {
    std::ofstream f(path);
    if (!f) {
      throw std::runtime_error("Cannot open for writing: " + path);
    }
    f.setf(std::ios::fixed);
    f.precision(6);
    // Relative image name, so the pair can be moved together.
    f << "image: " << image_name << "\n";
    f << "mode: trinary\n";
    f << "resolution: " << map_resolution_ << "\n";
    f << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
    f << "negate: 0\n";
    f << "occupied_thresh: 0.65\n";
    f << "free_thresh: 0.196\n";
    if (!f) {
      throw std::runtime_error("Failed while writing: " + path);
    }
  }

  std::string map_name_;
  std::string world_directory_;
  std::string input_subdir_;
  std::string output_subdir_;
  std::string input_pcd_override_;
  std::string output_basepath_override_;
  std::string output_name_;

  fs::path input_pcd_;
  fs::path output_base_;   // no extension

  bool use_statistical_filter_{false};
  int sor_mean_k_{30};
  double sor_std_ratio_{1.0};

  bool use_coarse_z_filter_{false};
  double coarse_z_min_{-5.0};
  double coarse_z_max_{5.0};

  bool use_ground_relative_slice_{false};
  std::string ground_method_{"csf"};

  bool csf_slope_smooth_{true};
  double csf_cloth_resolution_{0.5};
  int csf_rigidness_{2};
  double csf_class_threshold_{0.2};
  int csf_max_iterations_{500};
  double csf_time_step_{0.65};

  double ground_cell_size_{1.0};
  int ground_min_points_per_cell_{5};
  double ground_percentile_{0.05};
  double ground_max_step_{0.5};
  int ground_smooth_passes_{2};
  double ground_smooth_alpha_{0.5};

  bool use_voxel_filter_{false};
  bool voxel_before_height_filter_{false};
  double voxel_leaf_x_{0.05};
  double voxel_leaf_y_{0.05};
  double voxel_leaf_z_{0.05};
  int voxel_min_points_per_voxel_{1};

  double thre_z_min_{0.2};
  double thre_z_max_{2.0};
  bool flag_pass_through_{false};

  bool use_radius_filter_{true};
  double thre_radius_{0.5};
  int thres_point_count_{10};

  double map_resolution_{0.05};
  double padding_{0.5};
  bool unknown_as_unknown_{false};
  bool bounds_from_full_cloud_{true};
  bool save_intermediate_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    // Conversion happens in the constructor; there is nothing to spin on.
    auto node = std::make_shared<Pcd2Pgm>();
    RCLCPP_INFO(node->get_logger(), "Conversion complete.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("pcd2pgm"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}