#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
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
        private_node_.param<std::string>("output_topic", output_topic_, "/mavros/vision_pose/pose");
        private_node_.param<std::string>("frame_id", frame_id_, "map");

        // Kalibr and VINS use the D435i optical/IMU body axes: right-down-forward
        // (RDF). ROS and MAVROS expect forward-left-up (FLU). The VINS world
        // frame remains ENU: right/east, forward/north, up.
        flu_from_rdf_ << 0.0, 0.0, 1.0,
                        -1.0, 0.0, 0.0,
                         0.0, -1.0, 0.0;
        rdf_from_flu_ = Eigen::Quaterniond(flu_from_rdf_.transpose());

        publisher_ = node_.advertise<geometry_msgs::PoseStamped>(output_topic_, 10);
        subscriber_ = node_.subscribe(input_topic_, 10, &VinsPx4Bridge::odometryCallback, this);

        ROS_INFO_STREAM("VINS-PX4 bridge: " << input_topic_ << " -> " << output_topic_
                                             << " (PoseStamped, ENU/FLU, frame " << frame_id_ << ")");
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

        geometry_msgs::PoseStamped output;
        output.header.stamp = message->header.stamp;
        output.header.frame_id = frame_id_;
        output.pose.position = message->pose.pose.position;
        output.pose.orientation.w = world_from_flu.w();
        output.pose.orientation.x = world_from_flu.x();
        output.pose.orientation.y = world_from_flu.y();
        output.pose.orientation.z = world_from_flu.z();

        publisher_.publish(output);
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
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
