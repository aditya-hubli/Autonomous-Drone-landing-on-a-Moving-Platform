#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo
{
class DronePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    link_ = model_->GetLink("base_link");
    if (!link_) {
      link_ = model_->GetLinks()[0];
    }

    node_ = gazebo_ros::Node::Get(sdf);

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&DronePlugin::OnCmdVel, this, std::placeholders::_1));

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Control gains
    if (sdf->HasElement("linear_p_gain")) {
      linear_p_gain_ = sdf->Get<double>("linear_p_gain");
    }
    if (sdf->HasElement("angular_p_gain")) {
      angular_p_gain_ = sdf->Get<double>("angular_p_gain");
    }
    if (sdf->HasElement("max_force")) {
      max_force_ = sdf->Get<double>("max_force");
    }

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DronePlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(), "DronePlugin loaded for model: %s",
                model_->GetName().c_str());
  }

private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_vel_ = *msg;
    last_cmd_time_ = model_->GetWorld()->SimTime();
  }

  void OnUpdate()
  {
    auto now = model_->GetWorld()->SimTime();

    // Safety: zero cmd if no command received for 0.5s
    if ((now - last_cmd_time_).Double() > 0.5) {
      cmd_vel_ = geometry_msgs::msg::Twist();
    }

    double mass = link_->GetInertial()->Mass();

    // --- Gravity compensation ---
    link_->AddForce(ignition::math::Vector3d(0, 0, mass * 9.81));

    // --- Linear velocity control (world frame) ---
    auto current_vel = link_->WorldLinearVel();
    ignition::math::Vector3d target_vel(
      cmd_vel_.linear.x,
      cmd_vel_.linear.y,
      cmd_vel_.linear.z);

    auto vel_error = target_vel - current_vel;
    auto force = vel_error * mass * linear_p_gain_;

    // Clamp force
    if (force.Length() > max_force_) {
      force = force.Normalized() * max_force_;
    }
    link_->AddForce(force);

    // --- Yaw control ---
    auto current_ang_vel = link_->WorldAngularVel();
    double yaw_error = cmd_vel_.angular.z - current_ang_vel.Z();
    double torque_z = yaw_error * link_->GetInertial()->IZZ() * angular_p_gain_;
    link_->AddTorque(ignition::math::Vector3d(0, 0, torque_z));

    // --- Attitude stabilization (keep level) ---
    auto pose = link_->WorldPose();
    auto rot = pose.Rot();
    double roll = rot.Roll();
    double pitch = rot.Pitch();

    double stab_gain = 5.0 * mass;
    double roll_torque = -roll * stab_gain - current_ang_vel.X() * stab_gain * 0.5;
    double pitch_torque = -pitch * stab_gain - current_ang_vel.Y() * stab_gain * 0.5;
    link_->AddTorque(ignition::math::Vector3d(roll_torque, pitch_torque, 0));

    // --- Publish odometry at ~50Hz ---
    double dt = (now - last_odom_time_).Double();
    if (dt >= 0.02) {
      last_odom_time_ = now;

      nav_msgs::msg::Odometry odom;
      odom.header.stamp.sec = now.sec;
      odom.header.stamp.nanosec = now.nsec;
      odom.header.frame_id = "world";
      odom.child_frame_id = "drone/base_link";

      odom.pose.pose.position.x = pose.Pos().X();
      odom.pose.pose.position.y = pose.Pos().Y();
      odom.pose.pose.position.z = pose.Pos().Z();
      odom.pose.pose.orientation.x = rot.X();
      odom.pose.pose.orientation.y = rot.Y();
      odom.pose.pose.orientation.z = rot.Z();
      odom.pose.pose.orientation.w = rot.W();

      odom.twist.twist.linear.x = current_vel.X();
      odom.twist.twist.linear.y = current_vel.Y();
      odom.twist.twist.linear.z = current_vel.Z();
      odom.twist.twist.angular.x = current_ang_vel.X();
      odom.twist.twist.angular.y = current_ang_vel.Y();
      odom.twist.twist.angular.z = current_ang_vel.Z();

      odom_pub_->publish(odom);
    }
  }

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  gazebo_ros::Node::SharedPtr node_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  geometry_msgs::msg::Twist cmd_vel_;
  event::ConnectionPtr update_connection_;

  common::Time last_cmd_time_;
  common::Time last_odom_time_;

  double linear_p_gain_ = 5.0;
  double angular_p_gain_ = 3.0;
  double max_force_ = 30.0;
};

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)
}
