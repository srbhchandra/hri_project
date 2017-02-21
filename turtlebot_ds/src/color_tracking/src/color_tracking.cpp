/************************************************************
* This sample code demonstrates how to use cmvision package to track color blobs
***********************************************************/

#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>
#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/FollowerConfig.h"
#include <depth_image_proc/depth_traits.h>

/************************************************************
* Function Name: blobsCallBack
* Parameters: const cmvision::Blobs
* Returns: void
* Description: This is the callback function of the /blobs topic
***********************************************************/

double min_y_ = -0.5; /**< The minimum y position of the points in the box. */
double max_y_ =  0.5; /**< The maximum y position of the points in the box. */
double min_x_ = -0.5; /**< The minimum x position of the points in the box. */
double max_x_ =  0.5; /**< The maximum x position of the points in the box. */
double max_z_ =  1.2; /**< The maximum z position of the points in the box. */
double goal_z_=  0.6; /**< The distance away from the robot to hold the centroid */
double z_scale_ = 2.0; /**< The scaling factor for translational robot speed */
double x_scale_ = 7.0; /**< The scaling factor for rotational robot speed */
bool   enabled_ = true; /**< Enable/disable following; just prevents motor commands */

ros::Subscriber blobsSubscriber;
ros::Subscriber sub_;
ros::Publisher cmdpub_;

void blobsCallBack (const cmvision::Blobs& blobsIn)
{
/************************************************************
* These blobsIn.blobs[i].red, blobsIn.blobs[i].green, and blobsIn.blobs[i].blue values depend on the
* values those are provided in the colos.txt file.
* For example, the color file is like:
*
* [Colors]
* (255, 0, 0) 0.000000 10 RED
* (255, 255, 0) 0.000000 10 YELLOW
* [Thresholds]
* ( 127:187, 142:161, 175:197 )
* ( 47:99, 96:118, 162:175 )
*
* Now, if a red blob is found, then the blobsIn.blobs[i].red will be 255, and the others will be 0.
* Similarly, for yellow blob, blobsIn.blobs[i].red and blobsIn.blobs[i].green will be 255, and blobsIn.blobs[i].blue will be 0.
************************************************************/
  for (int i = 0; i < blobsIn.blob_count; i++) {
    if (blobsIn.blobs[i].red == 0 && blobsIn.blobs[i].green == 0 && blobsIn.blobs[i].blue == 255) {
      //ROS_INFO("%d: Blue blob found", i);
    }
  }
}

/*!
 * @bief Callback for point clouds.
 * Callback for depth images. It finds the centroid
 * of the points in a box in the center of the image. 
 * Publishes cmd_vel messages with the goal from the image.
 * @param cloud The point cloud message.
 */
void imageCallBack(const sensor_msgs::ImageConstPtr& depth_msg)
{

  // Precompute the sin function for each row and column
  uint32_t image_width = depth_msg->width;
  float x_radians_per_pixel = 60.0/57.0/image_width;
  float sin_pixel_x[image_width];
  for (int x = 0; x < image_width; ++x) {
    sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
  }

  uint32_t image_height = depth_msg->height;
  float y_radians_per_pixel = 45.0/57.0/image_width;
  float sin_pixel_y[image_height];
  for (int y = 0; y < image_height; ++y) {
    // Sign opposite x for y up values
    sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
  }

  //X,Y,Z of the centroid
  float x = 0.0;
  float y = 0.0;
  float z = 1e6;
  //Number of points observed
  unsigned int n = 0;

  //Iterate through all the points in the region and find the average of the position
  const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(float);
  for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
  {
   for (int u = 0; u < (int)depth_msg->width; ++u)
   {
     float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
     if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
     float y_val = sin_pixel_y[v] * depth;
     float x_val = sin_pixel_x[u] * depth;
     if ( y_val > min_y_ && y_val < max_y_ &&
          x_val > min_x_ && x_val < max_x_)
     {
       x += x_val;
       y += y_val;
       z = std::min(z, depth); //approximate depth as forward.
       n++;
     }
   }
  }
  
  //If there are points, find the centroid and calculate the command goal.
  //If there are no points, simply publish a stop goal.
  if (n>1000)
  {
    x /= n;
    y /= n;
    if(z > max_z_){
      ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot\n", z);
      if (enabled_)
      {
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      //cmd->angular.z = -0.4;
      cmdpub_.publish(cmd);
      }
      return;
    }

    ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d POINTS", x, y, z, n);
    
    if (enabled_)
    {
      ROS_INFO("In imageCallBack() - I should move");
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    	cmd->linear.x = (z - goal_z_) * z_scale_;
    	cmd->angular.z = -x * x_scale_;
      cmdpub_.publish(cmd);
    }
  }
  else
  {
    ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);

    if (enabled_)
    {
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      cmdpub_.publish(cmd);
    }
  }

}

int main(int argc, char **argv)
{
  //subscribe to /blobs topic
  ros::init(argc, argv, "color_tracker");
  ros::NodeHandle n;
  
  //cmdpub_ = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1); follower_velocity_smoother/raw_cmd_vel
  cmdpub_ = n.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/navi", 1);
  sub_    = n.subscribe<sensor_msgs::Image>("/camera/depth/image_rect", 1, imageCallBack);
  blobsSubscriber = n.subscribe("/blobs", 1, blobsCallBack);

  while(ros::ok())
  {
  	ros::spinOnce();
   }
}
