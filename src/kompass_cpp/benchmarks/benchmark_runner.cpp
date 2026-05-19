#include "benchmark_common.h"

// --- Project Includes ---
#include "datatypes/control.h"
#include "datatypes/path.h"
#include "datatypes/trajectory.h"
#include "utils/logger.h"

// --- Cost Evaluator ---
#include "utils/cost_evaluator.h"

// --- ROS 2 and Nav2 MPPI ---
#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_mppi_controller/critic_manager.hpp>
#include <nav2_mppi_controller/tools/parameters_handler.hpp>
#include <nav2_mppi_controller/models/state.hpp>
#include <nav2_mppi_controller/models/trajectories.hpp>
#include <nav2_mppi_controller/models/path.hpp>
#include <xtensor/xtensor.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_collision_monitor/polygon.hpp>
#include <nav2_collision_monitor/pointcloud.hpp>
#include <nav2_collision_monitor/scan.hpp>
#include <nav2_collision_monitor/types.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// --- Conditional Includes ---
#ifdef GPU
#include "mapping/local_mapper_gpu.h"
#include "utils/critical_zone_check_gpu.h"
#define PLATFORM_TAG "GPU_ACCELERATED"
#else
#include "mapping/local_mapper.h"
#include "utils/critical_zone_check.h"
#define PLATFORM_TAG "CPU_NATIVE"
#endif

#include <cstring> // for offsetof

using namespace Kompass;
using namespace Kompass::Benchmarks;

// =================================================================================
// 1. DATA GENERATORS
// =================================================================================

struct PointXYZ {
  float x, y, z, padding;
};

// Generates Heavy Trajectories for Cost Evaluator
std::unique_ptr<Control::TrajectorySamples2D>
generate_heavy_trajectory_samples(double predictionHorizon, double timeStep,
                                  int number_of_samples) {
  size_t number_of_points = static_cast<size_t>(predictionHorizon / timeStep);
  double v_1 = 1.0;
  double max_fluctuation = 0.5;

  auto samples = std::make_unique<Control::TrajectorySamples2D>(
      number_of_samples, number_of_points);

  // 1. Center Path
  Control::TrajectoryPath center_path(number_of_points);
  Control::TrajectoryVelocities2D center_vel(number_of_points);
  for (size_t i = 0; i < number_of_points; ++i) {
    center_path.add(i, Path::Point(timeStep * v_1 * i, 0.0, 0.0));
    if (i < number_of_points - 1)
      center_vel.add(i, Control::Velocity2D(v_1, 0.0, 0.0));
  }
  samples->push_back(center_vel, center_path);

  // 2. Generate Pairs
  int pairs = (number_of_samples - 1) / 2;
  double amp_step = max_fluctuation / (pairs > 0 ? pairs : 1);

  for (int p = 1; p <= pairs; ++p) {
    double current_amp = p * amp_step;

    // Linear Fluctuation
    Control::TrajectoryPath path_lin(number_of_points);
    Control::TrajectoryVelocities2D vel_lin(number_of_points);
    for (size_t i = 0; i < number_of_points; ++i) {
      double fluct_v = current_amp * std::sin(2 * M_PI * i / number_of_points);
      path_lin.add(
          i, Path::Point(timeStep * v_1 * i, timeStep * fluct_v * i, 0.0));
      if (i < number_of_points - 1)
        vel_lin.add(i, Control::Velocity2D(v_1, fluct_v, 0.0));
    }
    samples->push_back(vel_lin, path_lin);

    // Angular Fluctuation
    Control::TrajectoryPath path_ang(number_of_points);
    Control::TrajectoryVelocities2D vel_ang(number_of_points);
    for (size_t i = 0; i < number_of_points; ++i) {
      double fluct_ang =
          current_amp * std::cos(2 * M_PI * i / number_of_points);
      double x = timeStep * v_1 * i * std::cos(fluct_ang);
      double y = timeStep * v_1 * i * std::sin(fluct_ang);
      path_ang.add(i, Path::Point(x, y, 0.0));
      if (i < number_of_points - 1)
        vel_ang.add(i, Control::Velocity2D(v_1, 0.0, fluct_ang));
    }
    samples->push_back(vel_ang, path_ang);
  }
  return samples;
}

std::vector<int8_t> generate_heavy_pointcloud_bytes(size_t num_points) {
    // Create a temporary vector of strict types to ensure alignment
    std::vector<PointXYZ> temp_points(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        temp_points[i].x = (rand() % 2000) / 100.0f - 10.0f;
        temp_points[i].y = (rand() % 2000) / 100.0f - 10.0f;
        temp_points[i].z = (rand() % 300) / 100.0f;
        temp_points[i].padding = 0.0f;
    }

    // Memcpy into the byte buffer
    std::vector<int8_t> buffer(num_points * sizeof(PointXYZ));
    std::memcpy(buffer.data(), temp_points.data(), buffer.size());

    return buffer;
}

// Generates varied laserscan data for Mapping
void generate_mapping_scan(size_t num_points, std::vector<double> &ranges,
                           std::vector<double> &angles) {
  ranges.resize(num_points);
  angles.resize(num_points);
  double angle_step = (2.0 * M_PI) / num_points;
  for (size_t i = 0; i < num_points; ++i) {
    angles[i] = -M_PI + (i * angle_step);
    ranges[i] = 5.0 + 2.0 * std::sin(angles[i] * 20.0);
  }
}

// =================================================================================
// 2. MAIN RUNNER
// =================================================================================

class PublicCostmap2D : public nav2_costmap_2d::Costmap2D {
public:
    using Costmap2D::Costmap2D;
    
    template<class ActionType>
    inline void publicRaytraceLine(
        ActionType at, unsigned int x0, unsigned int y0, unsigned int x1,
        unsigned int y1,
        unsigned int max_length = UINT_MAX, unsigned int min_length = 0)
    {
        raytraceLine(at, x0, y0, x1, y1, max_length, min_length);
    }
};

std::vector<nav2_collision_monitor::Point> make_arc_polygon(double radius, double angle_deg, int segments = 30) {
    std::vector<nav2_collision_monitor::Point> poly;
    poly.push_back({0.0, 0.0});
    double half_angle = (angle_deg / 2.0) * M_PI / 180.0;
    double step = (2.0 * half_angle) / segments;
    for (int i = 0; i <= segments; ++i) {
        double a = -half_angle + i * step;
        poly.push_back({radius * std::cos(a), radius * std::sin(a)});
    }
    return poly;
}

class DummyPointCloud : public nav2_collision_monitor::PointCloud {
public:
    using PointCloud::PointCloud;
    void injectData(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        dataCallback(msg);
    }
};

class DummyScan : public nav2_collision_monitor::Scan {
public:
    using Scan::Scan;
    void injectData(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        dataCallback(msg);
    }
};

class DummyPolygon : public nav2_collision_monitor::Polygon {
public:
    using Polygon::Polygon;
    void injectPolygon(const std::vector<nav2_collision_monitor::Point>& points) {
        poly_ = points;
    }
};

int main(int argc, char *argv[]) {
  // 1. Setup Logging
  Kompass::setLogLevel(Kompass::LogLevel::INFO);
  
  // Setup ROS 2 for Nav2 MPPI Controller
  rclcpp::init(argc, argv);

  if (argc < 3) {
    LOG_ERROR("Usage: ./kompass_benchmark <platform_name> <output_json_path>");
    return 1;
  }

  std::string platform_alias = argv[1];
  std::string output_path = argv[2];
  std::string log_file_path = output_path + ".log";
  Kompass::setLogFile(log_file_path);

  std::vector<BenchmarkResult> results;

  LOG_INFO("===================================================");
  LOG_INFO("  KOMPASS CORE BENCHMARK SUITE");
  LOG_INFO("  Mode:  ", PLATFORM_TAG);
  LOG_INFO("  Target:", platform_alias);
  LOG_INFO("===================================================");

  // -------------------------------------------------------------------------
  // TEST 1: COST EVALUATOR
  // -------------------------------------------------------------------------
  {
    double predictionHorizon = 10.0;
    double timeStep = 0.01;
    int numTrajectories = 5001;

    std::vector<Path::Point> points{Path::Point(0.0, 0.0, 0.0),
                                    Path::Point(5.0, 0.0, 0.0),
                                    Path::Point(10.0, 0.0, 0.0)};
    Path::Path reference_path(points);
    reference_path.interpolate(0.01, Path::InterpolationType::LINEAR);
    reference_path.segment(1000.0, 1000);

    auto samples = generate_heavy_trajectory_samples(predictionHorizon,
                                                     timeStep, numTrajectories);

    Control::LinearVelocityControlParams x_p(1, 3, 5), y_p(1, 3, 5);
    Control::AngularVelocityControlParams a_p(3.14, 3, 5, 8);
    Control::ControlLimitsParams limits(x_p, y_p, a_p);
    
    // Create Nav2 MPPI Controller Node & Parameter Handler
    auto node = std::make_shared<nav2_util::LifecycleNode>("nav2_mppi_benchmark_node");
    
    // Declare critics parameter
    node->declare_parameter("nav2_mppi_benchmark_node.critics", std::vector<std::string>{"PathAlignCritic", "PathFollowCritic"});
    // find about 5 critics for collision check
    auto param_handler = std::make_unique<mppi::ParametersHandler>(node);
    
    // Initialize Critic Manager
    mppi::CriticManager critic_manager;
    auto dummy_costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("dummy_costmap");
    
    // Configure the manager (loads plugins and parameters)
    critic_manager.on_configure(node, "nav2_mppi_benchmark_node", dummy_costmap, param_handler.get());
    
    // Prepare Critic Data
    mppi::models::State state;
    mppi::models::Trajectories nav2_trajectories;
    mppi::models::Path nav2_path;
    
    nav2_trajectories.reset(numTrajectories, predictionHorizon / timeStep);
    nav2_path.reset(reference_path.getSize());
    
    // Fill reference path
    for (size_t i = 0; i < reference_path.getSize(); ++i) {
        nav2_path.x(i) = reference_path.getIndex(i).x();
        nav2_path.y(i) = reference_path.getIndex(i).y();
        nav2_path.yaws(i) = reference_path.getIndex(i).z();
    }
    
    // Fill trajectories from Kompass generated samples
    for (int i = 0; i < numTrajectories; ++i) {
        auto traj = samples->getIndex(i);
        for (size_t j = 0; j < traj.path.numPointsPerTrajectory_; ++j) {
            nav2_trajectories.x(i, j) = traj.path.x(j);
            nav2_trajectories.y(i, j) = traj.path.y(j);
            nav2_trajectories.yaws(i, j) = traj.path.z(j);
        }
    }
    
    xt::xtensor<float, 1> costs = xt::zeros<float>({(unsigned long)numTrajectories});
    float dt = timeStep;
    
    mppi::CriticData critic_data{
        state,
        nav2_trajectories,
        nav2_path,
        costs,
        dt,
        false, // fail_flag
        nullptr, // goal_checker
        nullptr, // motion_model
        std::make_optional<std::vector<bool>>(reference_path.getSize(), true), // path_pts_valid
        std::make_optional<size_t>(reference_path.getSize() - 1) // furthest_reached_path_point
    };

    auto workload = [&]() {
      // Nav2 MPPI Equivalent evaluation
      critic_manager.evalTrajectoriesScores(critic_data);
    };

    results.push_back(measure_performance("Nav2_MPPI_CriticManager_5k_Trajs", workload));
  }

  // -------------------------------------------------------------------------
  // TEST 2: MAPPING
  // -------------------------------------------------------------------------
  {
    int height = 400;
    int width = 400;
    float res = 0.05f;
    std::vector<double> ranges, angles;
    generate_mapping_scan(3600, ranges, angles);

#ifdef GPU
    // scan_size=3600 matches the generated scan so every ray reaches the
    // kernel. max_points_per_line=256 is the work-group size
    // and large enough for the longest ray here (range 3-7 m / res 0.05 m
    // = 60-140 cells) to reach its endpoint so OCCUPIED cells actually
    // get stamped
    Mapping::LocalMapperGPU mapper(height, width, res, {0.0, 0.0, 0.0}, 0.0,
                                   false, /*scan_size*/ 3600, 0.01, 2.0, 0.0,
                                   20.0, /*max_points_per_line*/ 256);
    auto workload = [&]() { mapper.scanToGrid(angles, ranges); };
#else
    float limit = width * res * std::sqrt(2);
    int maxPointsPerLine = static_cast<int>((limit / res) * 1.5);
    Mapping::LocalMapper mapper(height, width, res, {0.0, 0.0, 0.0}, 0.0, false,
                                0, 0.6, 0.9, 0.1, 0.1, 20.0, 0.2, 0.01, 2.0,
                                0.0, maxPointsPerLine, 10);
    auto workload = [&]() { mapper.scanToGridBaysian(angles, ranges); };
#endif

    results.push_back(measure_performance("Mapper_Dense_400x400", workload));

    // --- Nav2 Costmap2D Integration ---
    {
        // 1. Setup Costmap2D
        // 400x400 cells, 0.05 res. Origin at -10.0, -10.0 so sensor is centered.
        PublicCostmap2D costmap(400, 400, 0.05, -10.0, -10.0);
        
        // 2. Data Conversion Helper
        std::vector<std::pair<unsigned int, unsigned int>> hit_points;
        hit_points.reserve(ranges.size());
        
        unsigned int sensor_x, sensor_y;
        costmap.worldToMap(0.0, 0.0, sensor_x, sensor_y);

        for (size_t i = 0; i < ranges.size(); ++i) {
            double r = ranges[i];
            double a = angles[i];
            double px = r * std::cos(a);
            double py = r * std::sin(a);
            
            unsigned int mx, my;
            if (costmap.worldToMap(px, py, mx, my)) {
                hit_points.push_back({mx, my});
            }
        }

        struct ClearCell {
            unsigned char* cmap;
            ClearCell(unsigned char* cmap) : cmap(cmap) {}
            inline void operator()(unsigned int offset) {
                cmap[offset] = nav2_costmap_2d::FREE_SPACE;
            }
        };
        ClearCell clear_cell(costmap.getCharMap());

        auto nav2_workload = [&]() {
            for (const auto& pt : hit_points) {
                // Raytrace free space from sensor to hit point
                costmap.publicRaytraceLine(clear_cell, sensor_x, sensor_y, pt.first, pt.second);
                // Mark obstacle
                costmap.setCost(pt.first, pt.second, nav2_costmap_2d::LETHAL_OBSTACLE);
            }
        };
        
        results.push_back(measure_performance("Nav2_costmap_Dense_400x400", nav2_workload));
    }
  }

  // -------------------------------------------------------------------------
  // TEST 2b: MAPPING (Point Cloud → occupancy grid, GPU-only)
  //
  // Exercises the full pointcloud path: raw bytes → on-device
  // pointcloud_to_laserscan kernel → ray-cast kernel → occupancy grid.
  // Uses the same 100k synthetic cloud as CriticalZone_100k_Cloud so the
  // two benchmarks read identical input.
  // -------------------------------------------------------------------------
#ifdef GPU
  {
    const int height = 400;
    const int width = 400;
    const float res = 0.05f;
    const int scan_size = 3600;  // matches the laserscan benchmark
    const float angle_step = static_cast<float>(2.0 * M_PI / scan_size);
    const int max_points_per_line = 256;  // warp-multiple WG size, see TEST 2

    // Z-filter matches the critical-zone pointcloud benchmark so in-zone
    // points survive and out-of-zone ones get rejected on device.
    const float min_h = 0.1f;
    const float max_h = 2.0f;
    const float range_max = 20.0f;

    Mapping::LocalMapperGPU mapper(height, width, res, {0.0, 0.0, 0.0}, 0.0,
                                   /*isPointCloud*/ true, scan_size, angle_step,
                                   max_h, min_h, range_max,
                                   max_points_per_line);

    auto cloud_bytes = generate_heavy_pointcloud_bytes(100000);
    const int point_step = sizeof(PointXYZ);
    const int x_off = offsetof(PointXYZ, x);
    const int y_off = offsetof(PointXYZ, y);
    const int z_off = offsetof(PointXYZ, z);
    const int num_points = cloud_bytes.size() / point_step;
    const int pc_width = num_points;
    const int pc_height = 1;
    const int row_step = pc_width * point_step;

    auto workload = [&]() {
      mapper.scanToGrid(cloud_bytes, point_step, row_step, pc_height, pc_width,
                        static_cast<float>(x_off), static_cast<float>(y_off),
                        static_cast<float>(z_off));
    };

    results.push_back(
        measure_performance("Mapper_PointCloud_100k", workload));
  }
#endif



  // -------------------------------------------------------------------------
  // TEST 3: CRITICAL ZONE (Point Cloud)
  // -------------------------------------------------------------------------
  {
    auto shape = CollisionChecker::ShapeType::CYLINDER;
    std::vector<float> robotDim{0.51, 2.0};
    Eigen::Vector3f sensorPos{0.22, 0.0, 0.4};
    Eigen::Vector4f sensorRot{0, 0, 0.99, 0.0};
    float crit_angle = 160.0, crit_dist = 0.3, slow_dist = 0.6;
    std::vector<double> dummy_angles;
    dummy_angles.reserve(360);
    double angle_step = 2.0 * M_PI / 360.0;
    for (int i = 0; i < 360; ++i) {
      dummy_angles.push_back(i * angle_step);
    }
    auto cloud_bytes = generate_heavy_pointcloud_bytes(100000); // 100k points

    int point_step = sizeof(PointXYZ);
    int x_off = offsetof(PointXYZ, x);
    int y_off = offsetof(PointXYZ, y);
    int z_off = offsetof(PointXYZ, z);
    int num_points = cloud_bytes.size() / point_step;
    int width = num_points;
    int height = 1;
    int row_step = width * point_step;

#ifdef GPU
    CriticalZoneCheckerGPU checker(CriticalZoneChecker::InputType::POINTCLOUD,
                                   shape, robotDim, sensorPos, sensorRot,
                                   crit_angle, crit_dist, slow_dist,
                                   dummy_angles, 0.1, 2.0, 20.0);
#else
    CriticalZoneChecker checker(CriticalZoneChecker::InputType::POINTCLOUD,
                                shape, robotDim, sensorPos, sensorRot,
                                crit_angle, crit_dist, slow_dist, dummy_angles,
                                0.1, 2.0, 20.0);
#endif

    auto workload = [&]() {
      checker.check(cloud_bytes, point_step, row_step, height, width, x_off,
                    y_off, z_off, true);
    };

    results.push_back(measure_performance("CriticalZone_100k_Cloud", workload));

    // --- Nav2 Collision Monitor Integration (TEST 3) ---
    {
        auto node = std::make_shared<nav2_util::LifecycleNode>("cm_node_3");
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_buffer->setUsingDedicatedThread(true);

        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = node->get_clock()->now();
        tf_stamped.header.frame_id = "base_link";
        tf_stamped.child_frame_id = "sensor_link";
        tf_stamped.transform.translation.x = sensorPos.x();
        tf_stamped.transform.translation.y = sensorPos.y();
        tf_stamped.transform.translation.z = sensorPos.z();
        tf_stamped.transform.rotation.x = sensorRot.x();
        tf_stamped.transform.rotation.y = sensorRot.y();
        tf_stamped.transform.rotation.z = sensorRot.z();
        tf_stamped.transform.rotation.w = sensorRot.w();
        tf_buffer->setTransform(tf_stamped, "default_authority", true);

        auto stop_poly = std::make_shared<DummyPolygon>(node, "stop_poly", tf_buffer, "base_link", tf2::durationFromSec(0.0));
        stop_poly->injectPolygon(make_arc_polygon(0.81, 160.0));
        auto slow_poly = std::make_shared<DummyPolygon>(node, "slow_poly", tf_buffer, "base_link", tf2::durationFromSec(0.0));
        slow_poly->injectPolygon(make_arc_polygon(1.11, 160.0));

        auto source = std::make_shared<DummyPointCloud>(node, "cloud_source", tf_buffer, "base_link", "odom", tf2::durationFromSec(0.0), rclcpp::Duration(1, 0), false);
        node->declare_parameter("cloud_source.min_height", 0.1);
        node->declare_parameter("cloud_source.max_height", 2.0);
        node->declare_parameter("cloud_source.topic", "dummy");
        source->configure();

        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        msg->header.frame_id = "sensor_link";
        msg->header.stamp = node->get_clock()->now();
        msg->height = height;
        msg->width = width;
        msg->point_step = point_step;
        msg->row_step = row_step;
        msg->is_dense = true;
        msg->data.assign(cloud_bytes.begin(), cloud_bytes.end());
        
        sensor_msgs::msg::PointField x_f, y_f, z_f;
        x_f.name = "x"; x_f.offset = x_off; x_f.datatype = sensor_msgs::msg::PointField::FLOAT32; x_f.count = 1;
        y_f.name = "y"; y_f.offset = y_off; y_f.datatype = sensor_msgs::msg::PointField::FLOAT32; y_f.count = 1;
        z_f.name = "z"; z_f.offset = z_off; z_f.datatype = sensor_msgs::msg::PointField::FLOAT32; z_f.count = 1;
        msg->fields = {x_f, y_f, z_f};

        source->injectData(msg);

        auto cm_workload = [&]() {
            std::vector<nav2_collision_monitor::Point> points2d;
            source->getData(node->get_clock()->now(), points2d);
            int stop_pts = stop_poly->getPointsInside(points2d);
            int slow_pts = slow_poly->getPointsInside(points2d);
            (void)stop_pts; // Prevent unused warning
            (void)slow_pts;
        };
        results.push_back(measure_performance("Nav2_CollisionMonitor_100k_Cloud", cm_workload));
    }
  }

  // -------------------------------------------------------------------------
  // TEST 4: CRITICAL ZONE (LaserScan)
  // -------------------------------------------------------------------------
  {
    // TARGET: Force points to be in the "Slowdown Zone" (0.81m to 1.11m from
    // center) This ensures the CPU loop does not break early (collision) and
    // does not skip (safe).
    // -----------------------------------------------------------------------
    // Robot Radius = 0.51
    // Critical Limit = 0.51 + 0.3 = 0.81
    // Slowdown Limit = 0.51 + 0.6 = 1.11
    // Target Distance = 0.96m (Safe middle ground)

    std::vector<double> ranges, angles;
    size_t num_scan_points = 3600;
    ranges.resize(num_scan_points);
    angles.resize(num_scan_points);
    double angle_step = (2.0 * M_PI) / num_scan_points;

    double sensor_x = 0.22; // From sensorPos below
    double target_dist_sq = 0.96 * 0.96;

    // Generate ranges such that every point lands exactly at 'target_dist' from
    // robot center
    for (size_t i = 0; i < num_scan_points; ++i) {
      angles[i] = -M_PI + (i * angle_step);

      // Equation for distance R from robot center given sensor offset sx:
      // R^2 = (sx + r*cos_theta)^2 + (r*sin_theta)^2
      // R^2 = sx^2 + 2*sx*r*cos_theta + r^2
      // 0 = r^2 + (2*sx*cos_theta)*r + (sx^2 - R^2)

      double b = 2.0 * sensor_x * std::cos(angles[i]);
      double c = (sensor_x * sensor_x) - target_dist_sq;

      // Quadratic formula: r = (-b + sqrt(b^2 - 4ac)) / 2a.  (a=1)
      double discriminant = b * b - 4.0 * c;
      if (discriminant >= 0) {
        ranges[i] = (-b + std::sqrt(discriminant)) / 2.0;
      } else {
        ranges[i] = 10.0; // Fallback (should not happen with these params)
      }
    }

    std::vector<float> robotDim{0.51, 2.0};
    Eigen::Vector3f sensorPos{0.22, 0.0, 0.4};
    Eigen::Vector4f sensorRot{0, 0, 0.99, 0.0};

#ifdef GPU
    CriticalZoneCheckerGPU checker(CriticalZoneChecker::InputType::LASERSCAN,
                                   CollisionChecker::ShapeType::CYLINDER,
                                   robotDim, sensorPos, sensorRot, 160.0, 0.3,
                                   0.6, angles, 0.1, 2.0, 20.0);
#else
    CriticalZoneChecker checker(CriticalZoneChecker::InputType::LASERSCAN,
                                CollisionChecker::ShapeType::CYLINDER, robotDim,
                                sensorPos, sensorRot, 160.0, 0.3, 0.6, angles,
                                0.1, 2.0, 20.0);
#endif

    auto workload = [&]() { checker.check(ranges, true); };

    results.push_back(measure_performance("CriticalZone_Dense_Scan", workload));

    // --- Nav2 Collision Monitor Integration (TEST 4) ---
    {
        auto node = std::make_shared<nav2_util::LifecycleNode>("cm_node_4");
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_buffer->setUsingDedicatedThread(true);

        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = node->get_clock()->now();
        tf_stamped.header.frame_id = "base_link";
        tf_stamped.child_frame_id = "sensor_link";
        tf_stamped.transform.translation.x = sensorPos.x();
        tf_stamped.transform.translation.y = sensorPos.y();
        tf_stamped.transform.translation.z = sensorPos.z();
        tf_stamped.transform.rotation.x = sensorRot.x();
        tf_stamped.transform.rotation.y = sensorRot.y();
        tf_stamped.transform.rotation.z = sensorRot.z();
        tf_stamped.transform.rotation.w = sensorRot.w();
        tf_buffer->setTransform(tf_stamped, "default_authority", true);

        auto stop_poly = std::make_shared<DummyPolygon>(node, "stop_poly", tf_buffer, "base_link", tf2::durationFromSec(0.0));
        stop_poly->injectPolygon(make_arc_polygon(0.81, 160.0));
        auto slow_poly = std::make_shared<DummyPolygon>(node, "slow_poly", tf_buffer, "base_link", tf2::durationFromSec(0.0));
        slow_poly->injectPolygon(make_arc_polygon(1.11, 160.0));

        auto source = std::make_shared<DummyScan>(node, "scan_source", tf_buffer, "base_link", "odom", tf2::durationFromSec(0.0), rclcpp::Duration(1, 0), false);
        node->declare_parameter("scan_source.topic", "dummy");
        source->configure();

        auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        msg->header.frame_id = "sensor_link";
        msg->header.stamp = node->get_clock()->now();
        msg->angle_min = -M_PI;
        msg->angle_max = M_PI;
        msg->angle_increment = angle_step;
        msg->range_min = 0.1;
        msg->range_max = 20.0;
        
        msg->ranges.assign(ranges.begin(), ranges.end());
        source->injectData(msg);

        auto cm_workload = [&]() {
            std::vector<nav2_collision_monitor::Point> points2d;
            source->getData(node->get_clock()->now(), points2d);
            int stop_pts = stop_poly->getPointsInside(points2d);
            int slow_pts = slow_poly->getPointsInside(points2d);
            (void)stop_pts;
            (void)slow_pts;
        };
        results.push_back(measure_performance("Nav2_CollisionMonitor_Dense_Scan", cm_workload));
    }
  }

  save_results_to_json(platform_alias, results, output_path);
  LOG_INFO("Benchmark suite completed.");
  rclcpp::shutdown();

  return 0;
}
