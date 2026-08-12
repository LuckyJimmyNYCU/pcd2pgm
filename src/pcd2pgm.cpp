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
// Loads the PCD, keeps a height band with a PassThrough filter, optionally
// removes sparse points with a RadiusOutlierRemoval filter, rasterises what is
// left and writes a .pgm + .yaml pair that nav2_map_server can load directly.
// No topic is published; the node writes the files and exits.

#include <algorithm>
#include <cmath>
#include <cstdint>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

    // ---- optional early downsample ------------------------------------
    // Only when voxel_before_height_filter is true. This sees the full z
    // range of the cloud, so the voxel grid can get very large; the height
    // filter first is usually the better order.
    CloudT::Ptr work = cloud;
    if (use_voxel_filter_ && voxel_before_height_filter_) {
      work = voxelFilter(cloud);
      if (work->points.empty()) {
        throw std::runtime_error(
                "No points left after the voxel filter. Lower "
                "voxel_min_points_per_voxel or shrink voxel_leaf_*.");
      }
    }

    // ---- filter -------------------------------------------------------
    auto passed = passThrough(work);
    if (passed->points.empty()) {
      throw std::runtime_error(
              "No points left after the height filter. Check thre_z_min / "
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
    writeMap(cloud, obstacles);
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

    // Voxel downsampling. Applied to the raw cloud before every other
    // filter. leaf_z can be larger than leaf_x / leaf_y: the map is a 2D
    // raster, so vertical detail inside the height band is not needed.
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