/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *		 * Redistributions of source code must retain the above copyright
 *			 notice, this list of conditions and the following disclaimer.
 *		 * Redistributions in binary form must reproduce the above copyright
 *			 notice, this list of conditions and the following disclaimer in the
 *			 documentation and/or other materials provided with the distribution.
 *		 * Neither the name of the Willow Garage, Inc. nor the names of its
 *			 contributors may be used to endorse or promote products derived from
 *			 this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>
#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/FollowerConfig.h"
#include <depth_image_proc/depth_traits.h>
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>


#define MAX_AREA_LIST_SIZE (3)

namespace turtlebot_follower
{

/* The turtlebot follower nodelet.
/**
 * The turtlebot follower nodelet. Subscribes to point clouds
 * from the 3dsensor, processes them, and publishes command vel
 * messages.
 */
class TurtlebotFollower : public nodelet::Nodelet
{
public:
	/*!
	 * @brief The constructor for the follower.
	 * Constructor for the follower.
	*/
	TurtlebotFollower() : min_y_(-0.5), max_y_(0.5),
												min_x_(-0.5), max_x_(0.5),
												max_z_(0.8), goal_z_(0.8),
												z_scale_(1.0), x_scale_(5.0)
 	{

	}

	~TurtlebotFollower()
	{
		delete config_srv_;
	}

private:
	double min_y_; /**< The minimum y position of the points in the box. */
	double max_y_; /**< The maximum y position of the points in the box. */
	double min_x_; /**< The minimum x position of the points in the box. */
	double max_x_; /**< The maximum x position of the points in the box. */
	double max_z_; /**< The maximum z position of the points in the box. */
	double goal_z_; /**< The distance away from the robot to hold the centroid */
	double z_scale_; /**< The scaling factor for translational robot speed */
	double x_scale_; /**< The scaling factor for rotational robot speed */
	bool   enabled_; /**< Enable/disable following; just prevents motor commands */
	bool   enable_imagecb_movt;
	float  z, z_max;
	int    cent_x, prev_cent_x;
	int    max_curr_area, max_area, max_index, max_area_list[MAX_AREA_LIST_SIZE], area_counter;
	int    max_prev_area, max_prev_index;
	double object_found;
	double object_found_steps;
	double obstacle_found;
	double sum_linear_obstacle_found;
	int first_time_object_alignment;

	// Service for start/stop following
	ros::ServiceServer switch_srv_;

	// Dynamic reconfigure server
	dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>* config_srv_;

	/*!
	 * @brief OnInit method from node handle.
	 * OnInit method from node handle. Sets up the parameters
	 * and topics.
	 */
	virtual void onInit()
	{
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& private_nh = getPrivateNodeHandle();

		private_nh.getParam("min_y", min_y_);
		private_nh.getParam("max_y", max_y_);
		private_nh.getParam("min_x", min_x_);
		private_nh.getParam("max_x", max_x_);
		private_nh.getParam("max_z", max_z_);
		private_nh.getParam("goal_z", goal_z_);
		private_nh.getParam("z_scale", z_scale_);
		private_nh.getParam("x_scale", x_scale_);
		private_nh.getParam("enabled", enabled_);
 
		cmdpub_		= private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
//		markerpub_	= private_nh.advertise<visualization_msgs::Marker>("marker",1);
//		bboxpub_	= private_nh.advertise<visualization_msgs::Marker>("bbox",1);
		switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollower::changeModeSrvCb, this);

		sub_			 = nh.subscribe<sensor_msgs::Image>("depth/image_rect", 1, &TurtlebotFollower::imagecb, this);
		blobsSubscriber_ = nh.subscribe("/blobs", 1, &TurtlebotFollower::blobsCallBack, this);

		config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>(private_nh);
		dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>::CallbackType f 
					= boost::bind(&TurtlebotFollower::reconfigure, this, _1, _2);
		config_srv_->setCallback(f);

		z              =  1e6;
		z_max          =  0;
		cent_x         =  0;
		prev_cent_x    =  0;
		max_area       =  0;
		max_index      = -1;
		max_prev_area  =  0;
		max_prev_index = -1;
		max_curr_area  =  0;
		area_counter   =  0;
		object_found   =  0;//false;
		object_found_steps        = 1;//0.2;
		obstacle_found            = 0;
		sum_linear_obstacle_found = 0;
		enabled_            = true;
		enable_imagecb_movt = false;
		first_time_object_alignment = 1;
	}

	void reconfigure(turtlebot_follower::FollowerConfig &config, uint32_t level)
	{
		min_y_ = config.min_y;
		max_y_ = config.max_y;
		min_x_ = config.min_x;
		max_x_ = config.max_x;
		max_z_ = config.max_z;
		goal_z_ = config.goal_z;
		z_scale_ = config.z_scale;
		x_scale_ = config.x_scale;
	}

	void translate_and_rotate(double translation, double rotation_radians)
	{
		geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
		cmd->linear.x  = translation;
		cmd->angular.z = rotation_radians;
		ROS_INFO("translate_and_rotate:: linear.x = %f	angular.z = %f", cmd->linear.x, cmd->angular.z);
		cmdpub_.publish(cmd);
	}

	void blobsCallBack (const cmvision::Blobs& blobsIn)
	{
		int area = 0, i = 0;
		double linear_x = 0, angular_z = 0;

		linear_x = (z - goal_z_) * z_scale_;
		ROS_INFO("\nblobsCallBack:: Num Blob Count = %d,  linear_x = %f", blobsIn.blob_count, linear_x);

		if (first_time_object_alignment == 1)
			goto FIND_OBJECT;

		/* If obstacle found - rotate in opp direction and try to go ahead */
		if (linear_x <= 0.03)
		{
			ROS_INFO("---------------- Obstacle Found ---------------- ");
			object_found = 0;
			obstacle_found = 1;
			sum_linear_obstacle_found = 0;
			linear_x = std::max(linear_x, (double) 0);
			translate_and_rotate(linear_x, 0.4);
			//translate_and_rotate(0, 0.4);
			return;
		}

        /* Rotated away from obstacle. Now before trying to find to goal, try to move a bit ahead to go beyond obstacle. */
		if (obstacle_found == 1)
		{
			ROS_INFO("---------------- Obstacle Found before: move ahead ---------------- ");
			translate_and_rotate(linear_x, 0);
			sum_linear_obstacle_found += linear_x;
			/* Moved enough ahead. */
			if (sum_linear_obstacle_found > 2.5)
			{
				obstacle_found = 0;
			}
			return;
		}

		/*************** Attepted to navigate beyond obstacle. Now look for goal (object). ***************/
FIND_OBJECT:
		/* If goal (object) not found, just rotate. */
		/* It is possible the goal is in the line of view but it doesn't detect glabs eventhough they were
		   detected the previous frame. */
		if ( z > max_z_ || blobsIn.blob_count == 0)
		{
			object_found = 0;
			area_counter = 0;
			translate_and_rotate(0, -0.4);
			return;
		} 

		// Find max_area of blob - mostly the zeroth blob.
		max_area = 0;
		for (i = 0; i < blobsIn.blob_count; i++) 
		{
			if (blobsIn.blobs[i].red == 0 && blobsIn.blobs[i].green == 0 && blobsIn.blobs[i].blue == 255) 
			{
				area = (blobsIn.blobs[i].right - blobsIn.blobs[i].left) * (blobsIn.blobs[i].bottom - blobsIn.blobs[i].top);
				//area = blobsIn.blobs[i].area;
				if (area > max_area)
				{
					max_area = area;
					max_index = i;
				}
			}
		}
		ROS_INFO("Object partially found: area[%d] = %d", area_counter - 1, max_area);

		/* If goal already found, just navigate towards it. */
		if (object_found == 1)
		{	
			first_time_object_alignment = 0;

			/* if area is decreasing, then re-align */
			if (max_area < max_curr_area)
			{
				object_found = 0;
			} else
			{
				max_curr_area = max_area;
				if (z_max - z > 0.03)
				{
					ROS_INFO("Object fully found, translate ahead");
					translate_and_rotate(linear_x, 0);
					return;
				} else
				{
					ROS_INFO("****************** OBJECT REACHED ******************");
					translate_and_rotate(0, 0);
					return;
				}
			}
		}

		/* Object found - but only patially - ie blobsIn.blob_count > 0. */
 		max_area_list[area_counter++] = max_area;

 		if (area_counter == MAX_AREA_LIST_SIZE)
 		{
	 		max_area = 0;
			for (i = 0; i < MAX_AREA_LIST_SIZE; i++) 
			{
				if (max_area_list[i] > max_area)
				{
					max_area = max_area_list[i];
				}
			}
	 		max_prev_area = max_curr_area;
	 		max_curr_area = max_area;
			ROS_INFO("------------- Area changing from %d to %d", max_prev_area, max_curr_area);

			if ( max_curr_area > max_prev_area)
			{
				// Continue rotation and trying to find the complete object
				ROS_INFO("Continue rotation to find the complete object - area increasing from %d to %d", max_prev_area, max_curr_area);
				translate_and_rotate(0, -0.1);
				//translate_and_rotate(0, angular_z);
			} else
			{
				ROS_INFO("Object completely found");
				object_found = 1;
				translate_and_rotate(linear_x, 0.15);
			}

	 		area_counter = 0;
	 		return;
 		} else
 		{
			ROS_INFO("Continue rotation to find the complete object");
			translate_and_rotate(0, -0.1);
			return;
 		}
	}


	void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
	{
		uint32_t image_width  = depth_msg->width;
		uint32_t image_height = depth_msg->height;

		float x_radians_per_pixel = 60.0/57.0/image_width;
		float y_radians_per_pixel = 45.0/57.0/image_width;

		float sin_pixel_x[image_width], sin_pixel_y[image_height];
		const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
		float x = 0.0, y = 0.0;
		int i, j, n = 0; //Number of points observed
		int row_step = depth_msg->step / sizeof(float);

		// Precompute the sin function for each row and column
		for (i = 0; i < image_width; ++i) {
			sin_pixel_x[i] = sin((i - image_width/ 2.0)	* x_radians_per_pixel);
		}

		for (j = 0; j < image_height; ++j) {
			// Sign opposite x for y up values
			sin_pixel_y[j] = sin((image_height/ 2.0 - j) * y_radians_per_pixel);
		}

		//X,Y,Z of the centroid
		z	  = 1e6;
		z_max = 0;

		//Iterate through all the points in the region and find the average of the position
		for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
		{
			for (int u = 0; u < (int)depth_msg->width; ++u)
			{
				float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);

				if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) 
					continue;
				float y_val = sin_pixel_y[v] * depth;
				float x_val = sin_pixel_x[u] * depth;
				if ( y_val > min_y_ && y_val < max_y_ && x_val > min_x_ && x_val < max_x_)
				{
					x += x_val;
					y += y_val;
					z     = std::min(z, depth);     //approximate shortest depth as forward.
					z_max = std::max(z_max, depth); //approximate longest  depth as forward.
					n++;
				}
			}
		}

		//If there are points, find the centroid and calculate the command goal.
		//If there are no points, simply publish a stop goal.
		if (n>4000)
		{
			x /= n;
			y /= n;
			if(z > max_z_)
			{
				ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot\n", z);
//				publishMarker(x, y, z);
				if (enable_imagecb_movt)
					translate_and_rotate(0, -0.4);
			} else
			{
				ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d POINTS", x, y, z, n);
				if (enable_imagecb_movt)
					translate_and_rotate((z - goal_z_) * z_scale_, -x * x_scale_);
			}
		} else
		{
			ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);
//			publishMarker(x, y, z);
			if (enable_imagecb_movt)
				translate_and_rotate(0, 0);
		}
//		publishBbox();
	}

	bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
											 turtlebot_msgs::SetFollowState::Response& response)
	{
		if ((enabled_ == true) && (request.state == request.STOPPED))
		{
			ROS_INFO("Change mode service request: following stopped");
			cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
			enabled_ = false;
		}
		else if ((enabled_ == false) && (request.state == request.FOLLOW))
		{
			ROS_INFO("Change mode service request: following (re)started");
			enabled_ = true;
		}

		response.result = response.OK;
		return true;
	}

	void publishMarker(double x,double y,double z)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/camera_rgb_optical_frame";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		markerpub_.publish( marker );
	}

	void publishBbox()
	{
		double x = (min_x_ + max_x_)/2;
		double y = (min_y_ + max_y_)/2;
		double z = (0 + max_z_)/2;

		double scale_x = (max_x_ - x)*2;
		double scale_y = (max_y_ - y)*2;
		double scale_z = (max_z_ - z)*2;

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/camera_rgb_optical_frame";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 1;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = -y;
		marker.pose.position.z = z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = scale_x;
		marker.scale.y = scale_y;
		marker.scale.z = scale_z;
		marker.color.a = 0.5;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		bboxpub_.publish( marker );
	}

	ros::Subscriber sub_;
	ros::Publisher cmdpub_;
	ros::Publisher markerpub_;
	ros::Publisher bboxpub_;
	ros::Subscriber blobsSubscriber_;
};

PLUGINLIB_DECLARE_CLASS(turtlebot_follower, TurtlebotFollower, turtlebot_follower::TurtlebotFollower, nodelet::Nodelet);

}
