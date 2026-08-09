/********************************************************************************
 * File Name:   standard_message_publisher.h
 * Description: header for standard_message_publisher.cpp.
 *
 * License:     MIT License
 *
 * Note:        Publishes the ANELLO decoded fields as standard ROS message
 * 				types (sensor_msgs/Imu, sensor_msgs/NavSatFix, nav_msgs/Odometry)
 * 				in addition to the custom anello_interfaces messages published
 * 				by message_publisher.cpp.
 ********************************************************************************/

#ifndef STANDARD_MESSAGE_PUBLISHER_H
#define STANDARD_MESSAGE_PUBLISHER_H
#include "../main_anello_ros_driver.h"

#if COMPILE_WITH_ROS2
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

typedef rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_std_pub_t;
typedef rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_t;
typedef rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_t;

/*
 * Local ENU tangent-plane origin used to turn APINS lat/lon/alt into a position relative to
 * a fixed point. Anchored to the first INS fix received, since nav_msgs/Odometry has no
 * notion of geodetic coordinates.
 *
 * This is published with frame_id "map" by default (see map_frame_id in
 * main_anello_ros_driver.cpp), not "odom": the position comes from APINS, which already
 * incorporates GPS/RTK corrections, so it can jump when the RTK status changes (e.g. float ->
 * fixed) or after a GNSS reacquisition, rather than only drift smoothly. That matches REP-105's
 * definition of the map frame, not the odom frame (which dead-reckoning sensors like wheel/
 * visual odometry are expected to provide). If you add a localization stack (e.g.
 * robot_localization) with its own dead-reckoning-based odom -> base_link, this can still feed
 * it as the map -> odom correction.
 */
typedef struct
{
	bool initialized;
	double lat0_deg;
	double lon0_deg;
	double alt0_m;
} local_enu_origin_t;

/*
 * Parameters:
 * ax, ay, az						  : Accelerometer readings [g], ANELLO body frame (x-forward, y-right, z-down)
 * wx, wy, wz, wz_fog				  : Gyro rates [deg/s], ANELLO body frame
 * use_fog_gyro						  : If true, publish wz_fog (high-precision FOG) instead of wz (MEMS) as the z-axis rate
 * pub								  : Publisher used to publish the message
 * stamp							  : ROS time to stamp the message with
 * frame_id							  : frame_id to stamp the message with
 *
 * Notes:
 * Shared by both the APIMU and APIM1 decoders since a given ANELLO device only ever emits
 * one of the two. Orientation is not provided by either message, so orientation_covariance[0]
 * is set to -1 per the sensor_msgs/Imu convention. Units and body frame are converted from
 * ANELLO's g / deg/s / FRD to ROS REP-103's m/s^2 / rad/s / FLU.
 */
void publish_imu_raw(double ax, double ay, double az,
					  double wx, double wy, double wz, double wz_fog, bool use_fog_gyro,
					  imu_std_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);

/*
 * Parameters:
 * double *gps    : Same array passed to publish_gps()/publish_gp2() in message_publisher.h
 * pub			  : Publisher used to publish the message
 * stamp		  : ROS time to stamp the message with
 * frame_id		  : frame_id to stamp the message with
 */
void publish_navsatfix(double *gps, navsatfix_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);

/*
 * Parameters:
 * double *ins    : Same array passed to publish_ins() in message_publisher.h
 * pub			  : Publisher used to publish the message
 * stamp		  : ROS time to stamp the message with
 * frame_id		  : frame_id to stamp the message with
 *
 * Notes:
 * Publishes INS-derived orientation only (roll/pitch/heading). Angular velocity and linear
 * acceleration are not provided by APINS, so their covariance[0] is set to -1.
 */
void publish_ins_orientation(double *ins, imu_std_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);

/*
 * Parameters:
 * double *ins		 : Same array passed to publish_ins() in message_publisher.h
 * pub				 : Publisher used to publish the message
 * stamp			 : ROS time to stamp the message with
 * frame_id			 : frame_id to stamp the message with (default "map", see local_enu_origin_t)
 * child_frame_id	 : frame_id of the vehicle body (e.g. "base_link")
 * origin			 : Local ENU origin state, see local_enu_origin_t. Persists across calls.
 * publish_tf		 : If true, also broadcast the frame_id -> child_frame_id transform on /tf
 * tf_broadcaster	 : Broadcaster used to send the transform when publish_tf is true
 *
 * Notes:
 * Position is a local ENU tangent-plane approximation (flat-Earth, WGS84 mean radius) around
 * `origin` -- accurate to within the range needed for a few km of travel, not a substitute for
 * a proper geodetic projection over larger areas. Velocity is rotated into the body frame to
 * match the nav_msgs/Odometry convention that twist is expressed in child_frame_id.
 *
 * publish_tf defaults to on for out-of-the-box use (e.g. visualizing in rviz2), but should be
 * turned off if another node (e.g. robot_localization) already broadcasts this same transform --
 * two broadcasters publishing the same frame_id -> child_frame_id pair produces TF_REPEATED_DATA
 * warnings and undefined behavior for tf2 listeners.
 */
void publish_odometry(double *ins, odom_pub_t pub, rclcpp::Time stamp,
					   const std::string &frame_id, const std::string &child_frame_id,
					   local_enu_origin_t &origin, bool publish_tf,
					   tf2_ros::TransformBroadcaster &tf_broadcaster);

#endif
#endif
