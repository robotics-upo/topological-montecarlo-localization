#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include "ros/ros.h"
#include <limits>
#include <string>
#include <vector>
#include <fstream>

using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

double max_range = 10.0;

void decodeAndEmit(const sensor_msgs::CompressedImage& message);
void compressedImageCb(const sensor_msgs::CompressedImageConstPtr &c_im);

ros::Publisher image_pub;

int main(int argc, char **argv) 
{
  std::string camera("/up");

  ros::init(argc, argv, "test_plane_marker");
  
  cout << "Usage: " << argv[0] << " [<camera_name> = \"up\"] [<max_range> = 10] \n";

  if (argc > 1) {
    camera = string(argv[1]);   
  }
  cout << "Using camera: " << camera << endl;
  
  if (argc > 2) {
    max_range = atoi(argv[2]);
  } else
    max_range = 10.0;
  
  cout << "Maximum range: " << max_range << endl;
  
  ros::NodeHandle nh;
  std::string compressed_topic = camera + "/depth_registered/image_raw/compressedDepth";
  std::string uncompressed_topic = camera + "/rgb/image_raw/compressed";
  ros::Subscriber compressed_image_sub = nh.subscribe(compressed_topic, 1, &compressedImageCb);

  image_pub = nh.advertise<sensor_msgs::Image>(uncompressed_topic, 1);

  ros::spin();

  return 0;
}

void decodeAndEmit(const sensor_msgs::CompressedImage& message)
{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message.header;

  // Assign image encoding
  std::string image_encoding = message.format.substr(0, message.format.find(';'));
  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message.data.size() > sizeof(compressed_depth_image_transport::ConfigHeader))
  {

    // Read compression type from stream
    compressed_depth_image_transport::ConfigHeader compressionConfig;
    memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

    // Get compressed image data
    const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

    // Depth map decoding
    float depthQuantA, depthQuantB;

    // Read quantization parameters
    depthQuantA = compressionConfig.depthParam[0];
    depthQuantB = compressionConfig.depthParam[1];

    if (enc::bitDepth(image_encoding) == 32)
    {
      cv::Mat decompressed;
      try
      {
        // Decode image data
        decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0))
      {
        cv_ptr->image = Mat(rows, cols, CV_32FC1);
        
        sensor_msgs::Image img;
        cv_bridge::CvImage img_bridge;
        img_bridge.image = decompressed;
        img_bridge.encoding = cv_ptr->encoding;
        img_bridge.header = message.header;
        img_bridge.toImageMsg(img);

        image_pub.publish(img);
      }
    }
  }
}

void compressedImageCb(const sensor_msgs::CompressedImageConstPtr &c_im) {
    decodeAndEmit(*c_im);
}
