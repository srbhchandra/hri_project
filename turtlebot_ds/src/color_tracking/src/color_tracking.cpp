/************************************************************
* This sample code demonstrates how to use cmvision package to track color blobs
***********************************************************/

#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <ros/ros.h>

/************************************************************
* Function Name: blobsCallBack
* Parameters: const cmvision::Blobs
* Returns: void
* Description: This is the callback function of the /blobs topic
***********************************************************/
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
    //cout<<"%d: ";
    if (blobsIn.blobs[i].red == 0 && blobsIn.blobs[i].green == 0 && blobsIn.blobs[i].blue == 255) {
      ROS_INFO("%d: Blue blob found", i);
      // RED blob found: do something ...
    }
  }
}

int main(int argc, char **argv)
{
  //subscribe to /blobs topic
  ros::init(argc, argv, "color_tracker");
  ros::NodeHandle n;
  ros::Subscriber blobsSubscriber = n.subscribe("/blobs", 100, blobsCallBack);
  while(ros::ok())
  {
  	ros::spinOnce();
   }
}
