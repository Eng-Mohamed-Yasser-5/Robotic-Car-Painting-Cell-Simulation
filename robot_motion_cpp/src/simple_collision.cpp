#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <Eigen/Geometry>
#include <thread>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "spray_emitter.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("painting_simple_collision", opts);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spinner([&exec]() { exec.spin(); });

    moveit::planning_interface::MoveGroupInterface group(node, "arm");
    group.setEndEffectorLink("tool0");
    group.setMaxVelocityScalingFactor(0.3);


    // ---------- Add car as a MoveIt collision object ----------

    moveit::planning_interface::PlanningSceneInterface planning_scene;

    const std::string frame_id = "world";

    moveit_msgs::msg::CollisionObject car;
    car.header.frame_id = frame_id;
    car.id = "car_collision";

    // Load the car STL file.

    const std::string mesh_uri =
        "file://" +
        ament_index_cpp::get_package_share_directory("robot_description") +
        "/meshes/car/car.fbx";
    // const std::string mesh_uri = "file:///home/mohamed/Downloads/low-poly-mercedes-amg-gt-r/source/car.fbx";

    // STL dimensions is in meters, correct it with the the scale.
    Eigen::Vector3d scale(0.01, 0.01, 0.01);

    shapes::Mesh* car_mesh = shapes::createMeshFromResource(mesh_uri, scale);

    if (!car_mesh) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load car STL");
        rclcpp::shutdown();
        return 1;
    }

    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(car_mesh, mesh_msg);

    car.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));

    delete car_mesh; // clean the pointer after using it, we dont nead this any more :)

    // Position the car mesh like its position in the world.
    geometry_msgs::msg::Pose car_pose;
    car_pose.position.x = 0.0;
    car_pose.position.y = 1.4;
    car_pose.position.z = 0.05;
    Eigen::Quaterniond cq = 
                            Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitZ()) *   // yaw
                            Eigen::AngleAxisd( 0.00, Eigen::Vector3d::UnitY()) *   // pitch
                            Eigen::AngleAxisd( 1.57, Eigen::Vector3d::UnitX());    // roll

    car_pose.orientation.x = cq.x();
    car_pose.orientation.y = cq.y();
    car_pose.orientation.z = cq.z();
    car_pose.orientation.w = cq.w();


    car.mesh_poses.push_back(car_pose);
    car.operation = moveit_msgs::msg::CollisionObject::ADD;

    if (!planning_scene.applyCollisionObject(car)) {
        RCLCPP_ERROR(node->get_logger(),"Failed to add car to MoveIt planning scene");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(),"Car collision object added to planning scene");

    // surface of painting
    const double cx = -1.2, cy =  0.9; // center
    const double z = 0.8;
    const double length = 0.9;       // length in X axis
    const double width = 0.8;        // length in Y axis
    const double spacing = 0.10;       // scape between lines of trajectory
    const double step = 0.05;          // the space betwen points of the trajectory
    const double standoff = 0.25;      // space between the spray gun and the surface

    // Quaterniond of the robot end effector
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));

    // generate trajectory points (zig-zag)
    // i used Ai to code this part
    // its a little complex for a small task, so i used ai so i  can focaus more on the main parts
    std::vector<geometry_msgs::msg::Pose> waypoints;
    int rows = static_cast<int>(width / spacing) + 1;
    int cols = static_cast<int>(length / step) + 1;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int jj = (i % 2 == 0) ? j : cols - 1 - j;  
            geometry_msgs::msg::Pose p;
            p.position.y = cy - width / 2 + i * spacing;
            p.position.x = cx - length / 2 + jj * step;
            p.position.z = z + standoff;
            p.orientation.x = q.x();
            p.orientation.y = q.y();
            p.orientation.z = q.z();
            p.orientation.w = q.w();
            waypoints.push_back(p);
        }
    }
    RCLCPP_INFO(node->get_logger(), "waypoints: %zu", waypoints.size());


    // ============  Move to the first waypoint  ============
    group.setPoseTarget(waypoints.front());


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    spray_controller::SprayEmitter spray; // from code => spray_emitter.hpp

    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Cannot plan to the first waypoint");
    } else {
        auto result = group.execute(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(),"Failed to execute initial plan");
        }
        else 
        {
            // ============  Compute Cartesian painting path  ============
            moveit_msgs::msg::RobotTrajectory traj;

            double fraction = group.computeCartesianPath(waypoints, 0.01, traj, true);

            RCLCPP_INFO(node->get_logger(),"Cartesian path fraction = %.2f", fraction);

            if (fraction > 0.99) {
                spray.on();  // turning on the spray on the start of the path
                auto cartesian_result = group.execute(traj); //follow the trajectory
                spray.off(); // Turning off the spray omn the end of the path


                // back to rest after finishing
                std::vector<double> joint_values = {0.0, -1.57, 1.57, -3.14, 3.14, 0.0};

                group.setJointValueTarget(joint_values);
                auto planning_result = group.plan(plan);

                if (planning_result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_INFO(node->get_logger(), "Planning successful. Executing...");

                    auto execution_result = group.execute(plan);

                    if (execution_result == moveit::core::MoveItErrorCode::SUCCESS){
                        RCLCPP_INFO(node->get_logger(), "Motion completed successfully.");
                    }
                    else{
                        RCLCPP_ERROR(node->get_logger(), "Execution failed.");
                    }
                }
                else{
                    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
                }


            if (cartesian_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(),"Painting trajectory execution failed");
            }
            } else {
                RCLCPP_ERROR(node->get_logger(),"Path incomplete or blocked by collision");
            }
        }
    }
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}