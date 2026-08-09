/********************************************************************************
 * File Name:   standard_message_publisher.cpp
 * Description: contains functions for publishing standard ROS message types
 * 				(sensor_msgs/Imu, sensor_msgs/NavSatFix, nav_msgs/Odometry)
 * 				derived from the ANELLO decoded fields.
 *
 * License:     MIT License
 ********************************************************************************/

#include "standard_message_publisher.h"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace
{
	constexpr double DEG2RAD = M_PI / 180.0;
	constexpr double STANDARD_GRAVITY = 9.80665; // m/s^2 per g (used to convert ANELLO's g-scaled accel)
	constexpr double WGS84_MEAN_RADIUS_M = 6378137.0;

	// ANELLO reports orientation/velocity in NED (nav) / FRD (body), while ROS REP-103 expects
	// ENU (nav) / FLU (body). The two fixed rotations below convert between them; see
	// standard_message_publisher.h for the derivation.
	// q_ned_to_enu: 180 deg rotation about the North+East bisector axis (swaps X/Y, negates Z)
	const tf2::Quaternion Q_NED_TO_ENU(0.70710678118654752, 0.70710678118654752, 0.0, 0.0);
	// q_frd_to_flu: 180 deg rotation about the body X (forward) axis (negates Y/Z)
	const tf2::Quaternion Q_FRD_TO_FLU(1.0, 0.0, 0.0, 0.0);

	tf2::Quaternion ned_rpy_to_enu_flu_quaternion(double roll_rad, double pitch_rad, double heading_rad)
	{
		tf2::Quaternion q_ned_body;
		q_ned_body.setRPY(roll_rad, pitch_rad, heading_rad);

		tf2::Quaternion q_enu_body = Q_NED_TO_ENU * q_ned_body * Q_FRD_TO_FLU;
		q_enu_body.normalize();
		return q_enu_body;
	}
}

void publish_imu_raw(double ax, double ay, double az,
					  double wx, double wy, double wz, double wz_fog, bool use_fog_gyro,
					  imu_std_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
	sensor_msgs::msg::Imu msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = frame_id;

	// No orientation estimate is available from raw IMU messages
	msg.orientation_covariance[0] = -1.0;

	double wz_selected = use_fog_gyro ? wz_fog : wz;

	// FRD body -> FLU body: X unchanged, Y and Z negated
	msg.angular_velocity.x = wx * DEG2RAD;
	msg.angular_velocity.y = -wy * DEG2RAD;
	msg.angular_velocity.z = -wz_selected * DEG2RAD;

	msg.linear_acceleration.x = ax * STANDARD_GRAVITY;
	msg.linear_acceleration.y = -ay * STANDARD_GRAVITY;
	msg.linear_acceleration.z = -az * STANDARD_GRAVITY;

	// Covariances for angular_velocity/linear_acceleration are left at zero (unknown), since
	// ANELLO does not report a noise figure for these -- that is distinct from "not estimated".

	pub->publish(msg);
}

void publish_navsatfix(double *gps, navsatfix_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
	/*
	 * gps[2] = Latitude [deg]
	 * gps[3] = Longitude [deg]
	 * gps[4] = Alt_ellipsoid [m]
	 * gps[8] = Hacc [m]
	 * gps[9] = Vacc [m]
	 * gps[11] = FixType (0=No Fix, 2=2D Fix, 3=3D Fix, 5=Time only)
	 * gps[15] = RTK Fix Status (0=SPP, 1=RTK Float, 2=RTK Fix)
	 */
	sensor_msgs::msg::NavSatFix msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = frame_id;

	int fix_type = (int)gps[11];
	int rtk_status = (int)gps[15];

	if (fix_type == 0)
	{
		msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
	}
	else if (rtk_status == 2)
	{
		// RTK fixed (centimeter-level): closest standard match is a ground-based augmented fix
		msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
	}
	else
	{
		msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
	}
	msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

	msg.latitude = gps[2];
	msg.longitude = gps[3];
	msg.altitude = gps[4]; // height above the WGS84 ellipsoid, matches sensor_msgs/NavSatFix

	double hacc = gps[8];
	double vacc = gps[9];
	msg.position_covariance[0] = hacc * hacc; // East
	msg.position_covariance[4] = hacc * hacc; // North
	msg.position_covariance[8] = vacc * vacc; // Up
	msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

	pub->publish(msg);
}

void publish_ins_orientation(double *ins, imu_std_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
	/*
	 * ins[9]  = Roll [deg]
	 * ins[10] = Pitch [deg]
	 * ins[11] = Heading [deg]
	 */
	sensor_msgs::msg::Imu msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = frame_id;

	// APINS does not report raw rates/acceleration
	msg.angular_velocity_covariance[0] = -1.0;
	msg.linear_acceleration_covariance[0] = -1.0;

	tf2::Quaternion q = ned_rpy_to_enu_flu_quaternion(ins[9] * DEG2RAD, ins[10] * DEG2RAD, ins[11] * DEG2RAD);

	msg.orientation.x = q.x();
	msg.orientation.y = q.y();
	msg.orientation.z = q.z();
	msg.orientation.w = q.w();

	pub->publish(msg);
}

void publish_odometry(double *ins, odom_pub_t pub, rclcpp::Time stamp,
					   const std::string &frame_id, const std::string &child_frame_id,
					   local_enu_origin_t &origin)
{
	/*
	 * ins[3] = Latitude [deg]
	 * ins[4] = Longitude [deg]
	 * ins[5] = Alt_ellipsoid [m]
	 * ins[6] = Vn [m/s]
	 * ins[7] = Ve [m/s]
	 * ins[8] = Vd [m/s]
	 * ins[9] = Roll [deg]
	 * ins[10] = Pitch [deg]
	 * ins[11] = Heading [deg]
	 */
	if (!origin.initialized)
	{
		origin.lat0_deg = ins[3];
		origin.lon0_deg = ins[4];
		origin.alt0_m = ins[5];
		origin.initialized = true;
	}

	double lat0_rad = origin.lat0_deg * DEG2RAD;
	double north = (ins[3] - origin.lat0_deg) * DEG2RAD * WGS84_MEAN_RADIUS_M;
	double east = (ins[4] - origin.lon0_deg) * DEG2RAD * WGS84_MEAN_RADIUS_M * std::cos(lat0_rad);
	double up = ins[5] - origin.alt0_m;

	nav_msgs::msg::Odometry msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = frame_id;
	msg.child_frame_id = child_frame_id;

	msg.pose.pose.position.x = east;
	msg.pose.pose.position.y = north;
	msg.pose.pose.position.z = up;

	tf2::Quaternion q = ned_rpy_to_enu_flu_quaternion(ins[9] * DEG2RAD, ins[10] * DEG2RAD, ins[11] * DEG2RAD);
	msg.pose.pose.orientation.x = q.x();
	msg.pose.pose.orientation.y = q.y();
	msg.pose.pose.orientation.z = q.z();
	msg.pose.pose.orientation.w = q.w();

	// NED -> ENU velocity: (Vn, Ve, Vd) -> (east=Ve, north=Vn, up=-Vd)
	tf2::Vector3 v_enu(ins[7], ins[6], -ins[8]);

	// nav_msgs/Odometry.twist is expressed in child_frame_id (the body frame), so rotate the
	// world-frame (ENU) velocity into the body frame using the same orientation just computed.
	tf2::Vector3 v_body = tf2::quatRotate(q.inverse(), v_enu);
	msg.twist.twist.linear.x = v_body.x();
	msg.twist.twist.linear.y = v_body.y();
	msg.twist.twist.linear.z = v_body.z();

	// APINS does not report angular rate; twist.angular and all covariances are left at zero
	// (unknown) rather than -1, since nav_msgs/Odometry has no "do not use" convention.

	pub->publish(msg);
}
