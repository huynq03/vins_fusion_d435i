#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace
{
class VinsPx4Bridge
{
  public:
    VinsPx4Bridge() : private_node_("~")
    {
        private_node_.param<std::string>("input_topic", input_topic_, "/vins_estimator/odometry");
        private_node_.param<std::string>("output_topic", output_topic_, "/mavros/odometry/out");
        private_node_.param<std::string>("parent_frame_id", parent_frame_id_, "odom");
        private_node_.param<std::string>("child_frame_id", child_frame_id_, "base_link");

        // Kalibr and VINS use the D435i optical/IMU body axes: right-down-forward
        // (RDF). ROS and MAVROS expect forward-left-up (FLU). Keep the VINS
        // local world frame; its horizontal heading is arbitrary at startup.
        flu_from_rdf_ << 0.0, 0.0, 1.0,
                        -1.0, 0.0, 0.0,
                         0.0, -1.0, 0.0;
        rdf_from_flu_ = Eigen::Quaterniond(flu_from_rdf_.transpose());

        publisher_ = node_.advertise<nav_msgs::Odometry>(output_topic_, 10);
        subscriber_ = node_.subscribe(input_topic_, 10, &VinsPx4Bridge::odometryCallback, this);

        ROS_INFO_STREAM("VINS-PX4 bridge: " << input_topic_ << " -> " << output_topic_
                                             << " (Odometry, parent " << parent_frame_id_
                                             << ", child " << child_frame_id_
                                             << ", MAVLink LOCAL_FRD/BODY_FRD)");
    }

  private:
    void odometryCallback(const nav_msgs::OdometryConstPtr &message)
    {
        Eigen::Quaterniond world_from_rdf(message->pose.pose.orientation.w,
                                          message->pose.pose.orientation.x,
                                          message->pose.pose.orientation.y,
                                          message->pose.pose.orientation.z);
        const double quaternion_norm = world_from_rdf.norm();
        if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-9)
        {
            ROS_WARN_THROTTLE(1.0, "Dropping VINS odometry with an invalid orientation");
            return;
        }

        world_from_rdf.normalize();
        Eigen::Quaterniond world_from_flu = world_from_rdf * rdf_from_flu_;
        world_from_flu.normalize();

        const Eigen::Vector3d position_world(message->pose.pose.position.x,
                                             message->pose.pose.position.y,
                                             message->pose.pose.position.z);
        if (!position_world.allFinite())
        {
            ROS_WARN_THROTTLE(1.0, "Dropping VINS odometry with an invalid position");
            return;
        }

        if (message->header.stamp.isZero())
            ROS_WARN_THROTTLE(1.0, "VINS odometry timestamp is zero");

        nav_msgs::Odometry output;
        output.header.stamp = message->header.stamp;
        output.header.frame_id = parent_frame_id_;
        output.child_frame_id = child_frame_id_;
        output.pose.pose.position = message->pose.pose.position;
        output.pose.pose.orientation.w = world_from_flu.w();
        output.pose.pose.orientation.x = world_from_flu.x();
        output.pose.pose.orientation.y = world_from_flu.y();
        output.pose.pose.orientation.z = world_from_flu.z();
        output.pose.covariance = message->pose.covariance;

        // nav_msgs/Odometry follows REP-147: twist is expressed in child_frame_id.
        // VINS publishes linear velocity in its world frame and angular velocity
        // in its RDF IMU/body frame. Convert both to ROS body FLU before MAVROS
        // performs the final FLU -> BODY_FRD conversion.
        const Eigen::Vector3d velocity_world(message->twist.twist.linear.x,
                                             message->twist.twist.linear.y,
                                             message->twist.twist.linear.z);
        const Eigen::Vector3d angular_velocity_rdf(message->twist.twist.angular.x,
                                                   message->twist.twist.angular.y,
                                                   message->twist.twist.angular.z);

        if (velocity_world.allFinite())
        {
            const Eigen::Vector3d velocity_rdf = world_from_rdf.inverse() * velocity_world;
            const Eigen::Vector3d velocity_flu = flu_from_rdf_ * velocity_rdf;
            output.twist.twist.linear.x = velocity_flu.x();
            output.twist.twist.linear.y = velocity_flu.y();
            output.twist.twist.linear.z = velocity_flu.z();
        }
        else
        {
            setLinearVelocityNaN(output);
        }

        if (angular_velocity_rdf.allFinite())
        {
            const Eigen::Vector3d angular_velocity_flu = flu_from_rdf_ * angular_velocity_rdf;
            output.twist.twist.angular.x = angular_velocity_flu.x();
            output.twist.twist.angular.y = angular_velocity_flu.y();
            output.twist.twist.angular.z = angular_velocity_flu.z();
        }
        else
        {
            setAngularVelocityNaN(output);
        }
        output.twist.covariance = message->twist.covariance;

        publisher_.publish(output);
    }

    static void setLinearVelocityNaN(nav_msgs::Odometry &message)
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        message.twist.twist.linear.x = nan;
        message.twist.twist.linear.y = nan;
        message.twist.twist.linear.z = nan;
    }

    static void setAngularVelocityNaN(nav_msgs::Odometry &message)
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        message.twist.twist.angular.x = nan;
        message.twist.twist.angular.y = nan;
        message.twist.twist.angular.z = nan;
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::string input_topic_;
    std::string output_topic_;
    std::string parent_frame_id_;
    std::string child_frame_id_;
    Eigen::Matrix3d flu_from_rdf_;
    Eigen::Quaterniond rdf_from_flu_;
};
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_px4_bridge");
    VinsPx4Bridge bridge;
    ros::spin();
    return 0;
}
