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
#include <string>

typedef rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_std_pub_t;
typedef rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_t;
typedef rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_t;

/*
 * Local ENU tangent-plane origin used to turn APINS lat/lon/alt into an "odom" frame
 * position. Anchored to the first INS fix received: nav_msgs/Odometry has no notion of
 * geodetic coordinates, and per REP-105 the odom frame only needs to be locally accurate,
 * not tied to a global datum.
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
 * frame_id			 : frame_id (the "odom" frame) to stamp the message with
 * child_frame_id	 : frame_id of the vehicle body (e.g. "base_link")
 * origin			 : Local ENU origin state, see local_enu_origin_t. Persists across calls.
 *
 * Notes:
 * Position is a local ENU tangent-plane approximation (flat-Earth, WGS84 mean radius) around
 * `origin` -- accurate to within the range needed for a few km of travel, not a substitute for
 * a proper geodetic projection over larger areas. Velocity is rotated into the body frame to
 * match the nav_msgs/Odometry convention that twist is expressed in child_frame_id.
 */
void publish_odometry(double *ins, odom_pub_t pub, rclcpp::Time stamp,
					   const std::string &frame_id, const std::string &child_frame_id,
					   local_enu_origin_t &origin);

#endif
#endif
