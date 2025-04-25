//
// Created by ubuntu on 2024-12-21.
// Tong Qin: qintong@sjtu.edu.cn
//
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "visualization_msgs/Marker.h"
#include "string.h"
#include <tf/transform_broadcaster.h>


ros::Publisher pub_path, pub_odometry;
nav_msgs::Path path;
ros::Publisher meshPub;


void Publish(const double &time, const double &x, const double &y, const double &yaw) {

    // convert 2D x, y, yaw to 3D x, y, z and rotation matrix
    Eigen::Vector3d position = Eigen::Vector3d(x, y, 0);
    Eigen::Matrix3d R;
    R << cos(yaw), - sin(yaw), 0,
    sin(yaw), cos(yaw), 0,
    0, 0, 1;
    Eigen::Quaterniond q(R);

    // pub odometry
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(time);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    pub_odometry.publish(odometry);

    // pub path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(time);
    pose_stamped.pose = odometry.pose.pose;
    path.poses.push_back(pose_stamped);
    if (path.poses.size() > 200) {
        path.poses.erase(path.poses.begin());
    }
    pub_path.publish(path);

    // pub tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion tf_q;
    transform.setOrigin(tf::Vector3(position(0),
                                    position(1),
                                    position(2)));
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    transform.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform, odometry.header.stamp,
                                          "world", "vehicle"));

    // pub Mesh model
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = std::string("world");
    meshROS.header.stamp = ros::Time(time);
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    Eigen::Matrix3d rot_mesh;
    rot_mesh << -1, 0, 0, 0, 0, 1, 0, 1, 0;
    Eigen::Quaterniond q_mesh;
    q_mesh = q * rot_mesh;
    Eigen::Vector3d t_mesh = R * Eigen:: Vector3d(1.5, 0, 0) + position;
    meshROS.pose.orientation.w = q_mesh.w();
    meshROS.pose.orientation.x = q_mesh.x();
    meshROS.pose.orientation.y = q_mesh.y();
    meshROS.pose.orientation.z = q_mesh.z();
    meshROS.pose.position.x = t_mesh(0);
    meshROS.pose.position.y = t_mesh(1);
    meshROS.pose.position.z = t_mesh(2);
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.0;
    std::string mesh_resource = "package://vehicle_visualization/meshes/car.dae";
    meshROS.mesh_resource = mesh_resource;
    meshPub.publish(meshROS);
}

void GeneratePose(const double &time, double &x, double &y, double &yaw) {
    // Your code here
    // Modify the function of x, y, and yaw, so that the vehicle can drive the shape you want.
    x = 10 * sin(time / 10 * M_PI);
    y = 10 * cos(time / 10 * M_PI) - 10;
    double vx = (1.0 / 10 * M_PI) * 10 * cos(time / 10 * M_PI);
    double vy = (1.0 / 10 * M_PI) * 10 * (-1) * sin(time / 10 * M_PI);
    yaw = atan2(vy , vx);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_visualization_node");
    ros::NodeHandle n("~");

    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    meshPub   = n.advertise<visualization_msgs::Marker>("vehicle", 100, true);
    path.header.frame_id = "world";

    ros::Rate loop_rate(10);
    double time = 0;
    double x = 0, y = 0, yaw = 0;
    while (ros::ok())
    {
        GeneratePose(time, x, y, yaw);
        Publish(time, x,y ,yaw);
        ros::spinOnce();
        loop_rate.sleep();
        time += 0.1;
    }
    ros::spin();
}
