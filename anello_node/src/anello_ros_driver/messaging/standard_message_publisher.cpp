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
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace
{
	constexpr double DEG2RAD = M_PI / 180.0;
	constexpr double STANDARD_GRAVITY = 9.80665; // m/s^2 per g (used to convert ANELLO's g-scaled accel)
	constexpr double WGS84_MEAN_RADIUS_M = 6378137.0;

	// GPS_Time (per the ANELLO Developer Manual) is nanoseconds since the GPS epoch,
	// 1980-01-06T00:00:00 UTC, which is exactly this many seconds after the Unix epoch.
	constexpr double GPS_TO_UNIX_EPOCH_OFFSET_S = 315964800.0;
	// GPS time has no leap seconds; UTC (and therefore Unix time) does. This is the current
	// constant offset (unchanged since the last leap second, 2016-12-31) -- would need updating
	// if a new leap second is ever inserted.
	constexpr double GPS_UTC_LEAP_SECONDS = 18.0;

	// TAI is always exactly this many seconds ahead of GPS time -- unlike GPS-UTC, this never
	// changes with new leap seconds, since neither TAI nor GPS time has leap seconds; UTC does.
	constexpr double TAI_GPS_OFFSET_S = 19.0;

	// If the ANELLO unit has gPTP enabled -- in either master or slave role -- APINS's GPS_Time
	// field is overwritten with PTP time instead of true GPS time (confirmed against real
	// hardware, in both roles; APGPS/APGP2/APHDG are unaffected either way). As master, the unit
	// has no external clock to slave to, so it most likely derives its own PTP/TAI clock from its
	// own GPS time, and that derived clock is what ends up in APINS's GPS_Time. PTP's default
	// profile (IEEE 1588) is TAI-referenced nanoseconds since 1970-01-01T00:00:00 TAI -- a
	// different epoch than GPS's (1980-01-06) *and* a different reference timescale (TAI, not
	// UTC), so converting PTP time to the GPS-time equivalent needs both the epoch shift and the
	// TAI-GPS offset: PTP_ns = GPS_ns + (epoch shift + TAI-GPS offset), i.e. GPS_ns = PTP_ns -
	// (GPS_TO_UNIX_EPOCH_OFFSET_S + TAI_GPS_OFFSET_S).
	constexpr double PTP_TO_GPS_EPOCH_OFFSET_S = GPS_TO_UNIX_EPOCH_OFFSET_S + TAI_GPS_OFFSET_S;

	// Converts a GPS_Time value (ns since the GPS epoch) into a message header stamp. Falls back
	// to `fallback` (typically the node's own clock) if gps_time_ns isn't populated yet (<= 0),
	// e.g. before the receiver has ever tracked a satellite.
	builtin_interfaces::msg::Time gps_time_ns_to_header_stamp(double gps_time_ns, const rclcpp::Time &fallback)
	{
		if (gps_time_ns <= 0.0)
		{
			return fallback;
		}

		double unix_ns = gps_time_ns + (GPS_TO_UNIX_EPOCH_OFFSET_S - GPS_UTC_LEAP_SECONDS) * 1.0e9;

		builtin_interfaces::msg::Time stamp;
		stamp.sec = static_cast<int32_t>(std::floor(unix_ns / 1.0e9));
		stamp.nanosec = static_cast<uint32_t>(unix_ns - static_cast<double>(stamp.sec) * 1.0e9);
		return stamp;
	}

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

double ins_gps_time_ns(double ins_time_ns, bool ins_gps_time_is_ptp)
{
	if (!ins_gps_time_is_ptp || ins_time_ns <= 0.0)
	{
		return ins_time_ns;
	}

	return ins_time_ns - PTP_TO_GPS_EPOCH_OFFSET_S * 1.0e9;
}

void publish_imu_raw(double ax, double ay, double az,
					  double wx, double wy, double wz, double wz_fog, bool use_fog_gyro,
					  double gps_time_ns, imu_std_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
	sensor_msgs::msg::Imu msg;
	msg.header.stamp = gps_time_ns_to_header_stamp(gps_time_ns, stamp);
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
	msg.header.stamp = gps_time_ns_to_header_stamp(gps[1], stamp);
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

void publish_ins_navsatfix(double *ins, navsatfix_pub_t pub, double gps_time_ns, rclcpp::Time stamp,
							const std::string &frame_id)
{
	/*
	 * ins[2] = INS Status (255=uninitialized, 0=Attitude only, 1=Pos and Att, 2=Pos Hdg Att,
	 *                      3=RTK Float, 4=RTK Fix)
	 * ins[3] = Latitude [deg]
	 * ins[4] = Longitude [deg]
	 * ins[5] = Alt_ellipsoid [m]
	 */
	sensor_msgs::msg::NavSatFix msg;
	msg.header.stamp = gps_time_ns_to_header_stamp(gps_time_ns, stamp);
	msg.header.frame_id = frame_id;

	int ins_status = (int)ins[2];

	if (ins_status == 4)
	{
		// RTK fixed (centimeter-level): closest standard match is a ground-based augmented fix
		msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
	}
	else if (ins_status == 1 || ins_status == 2 || ins_status == 3)
	{
		msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
	}
	else
	{
		// 255 (uninitialized) or 0 (attitude only): no position solution yet
		msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
	}
	msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

	msg.latitude = ins[3];
	msg.longitude = ins[4];
	msg.altitude = ins[5]; // height above the WGS84 ellipsoid, matches sensor_msgs/NavSatFix

	// Unlike APGPS, APINS reports no accuracy figures (no hacc/vacc equivalent), so there is
	// nothing to build a real covariance estimate from.
	msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

	pub->publish(msg);
}

void publish_odometry(double *ins, odom_pub_t pub, double gps_time_ns, rclcpp::Time stamp,
					   const std::string &frame_id, const std::string &child_frame_id,
					   local_enu_origin_t &origin, bool publish_tf,
					   tf2_ros::TransformBroadcaster &tf_broadcaster)
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

	builtin_interfaces::msg::Time header_stamp = gps_time_ns_to_header_stamp(gps_time_ns, stamp);

	nav_msgs::msg::Odometry msg;
	msg.header.stamp = header_stamp;
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

	if (publish_tf)
	{
		geometry_msgs::msg::TransformStamped tf_msg;
		tf_msg.header.stamp = header_stamp;
		tf_msg.header.frame_id = frame_id;
		tf_msg.child_frame_id = child_frame_id;

		tf_msg.transform.translation.x = east;
		tf_msg.transform.translation.y = north;
		tf_msg.transform.translation.z = up;
		tf_msg.transform.rotation = msg.pose.pose.orientation;

		tf_broadcaster.sendTransform(tf_msg);
	}
}
