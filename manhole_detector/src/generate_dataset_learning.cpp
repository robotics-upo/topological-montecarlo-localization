#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif
// #include <compressed_image_transport/codec.h>
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include "ros/ros.h"
#include <limits>
#include <string>
#include <vector>
#include <fstream>

#include <functions/functions.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

double max_range = 10.0;

void decodeAndWriteCompressed(const sensor_msgs::CompressedImage& message, ofstream &file, float rotate, float delta_rot, bool positive = false);

void writeImage(const cv::Mat &image, ofstream &file);

int main(int argc, char **argv) 
{
  
  rosbag::Bag bag;
  std::string camera("/up");
  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <bag file> <input_file> [<camera_name> = \"up\"] [<skip first n images = 0>] [<max_range = 10>\n";
    return -1;
  }
  if (argc > 3) {
    camera = string(argv[3]);
    cout << "Using camera: " << camera << endl;
  }
  std::string filename(argv[2]);
  
  int ignore = -1;
  if (argc > 4) {
    ignore = atoi(argv[4]);
    cout << "Ignoring ids with less than: " << ignore << endl;
  }
  if (argc > 5) {
    max_range = atoi(argv[4]);
    
  } else
    max_range = 10.0;
  
  cout << "Maximum range: " << max_range << endl;
  
  std::vector<std::vector <double> > M;
  functions::getMatrixFromFile(filename, M);
  
  std::string compressed_topic = camera + "/depth_registered/image_raw/compressedDepth";
  std::string rgb_topic = camera + "/rgb/image_raw/compressed";
  try {
    ofstream pos_depth_file("positive_depth");
    ofstream neg_depth_file("negative_depth");
    bag.open(string(argv[1]), rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(compressed_topic);
    topics.push_back(rgb_topic);
    

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    bool initialized = false;
    bool _positive = false;
    bool ignoring = true;
    foreach(rosbag::MessageInstance const m, view)
    {
      sensor_msgs::CompressedImage::Ptr im = m.instantiate<sensor_msgs::CompressedImage>();
      if (im != NULL) {
	if (m.getTopic() == rgb_topic) {
	  int seq = im->header.seq;
	  
	  if (seq > ignore) {
	    ignoring = false;
	    
	  } else
	    continue;
	  
	  cout << "ID = " << seq << "\n";
	  
	  _positive = false;
	  for (unsigned int i=0; i < M.size(); i++) {
	    
	    if (M[i][0] <= seq and M[i][1] >= seq) {
	      _positive = true;
	    }
	  }
	} 
	else 
	{
	  if (ignoring)
	    continue;
	  
	  if (_positive) {
	    decodeAndWriteCompressed(*im, pos_depth_file, 20, 10, true); 
	  } else {
	    decodeAndWriteCompressed(*im, neg_depth_file, 20, 10, false);
	  }
	} 
      }
    }

    bag.close();
  } catch (exception &e) {
    cerr << "Exception while manipulating the bag  " << argv[1] << endl;
    cerr << "Content " << e.what() << endl;
    return -2;
  }

  
  return 0;
}

void decodeAndWriteCompressed(const sensor_msgs::CompressedImage& message, ofstream &file, float rotate, float delta_rot, bool positive)
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
//         return sensor_msgs::Image::Ptr();
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0))
      {
        cv_ptr->image = Mat(rows, cols, CV_32FC1);

        // Depth conversion
        MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                            itDepthImg_end = cv_ptr->image.end<float>();
        MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                          itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
          // check for NaN & max depth
          if (*itInvDepthImg)
          {
            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
          }
          else
          {
//             *itDepthImg = std::numeric_limits<float>::quiet_NaN();
               *itDepthImg = max_range;
          }
        }
        
        if(positive) {
	  Mat rotated(rows, cols, CV_32FC1);
	  for (float rot =-rotate;rot < rotate + 1.0; rot += delta_rot) {
	    
	    //get the affine transformation matrix
	    Mat matRotation = getRotationMatrix2D( Point(decompressed.cols / 2, decompressed.rows / 2), rot, 1 );
	    warpAffine( decompressed, rotated, matRotation, decompressed.size() );
	    
	    Mat downsample(rows/4, cols/4, CV_32FC1);
	    cv::resize(rotated, downsample, Size(), 0.25, 0.25);
	    
	    writeImage(downsample, file);
	    flip(downsample, downsample, 0);
	    writeImage(downsample, file);
	    flip(downsample, downsample, -1);
	    writeImage(downsample, file);
	    
	    float delta = rows*0.25*0.15;
	    for (float offset = -delta; offset < delta + 0.01;offset += delta) {
	      if (fabs(offset) < 1e-2) 
		continue; // Ignore the case with no translation
	      Mat downsample(rows/4, cols/4, CV_32FC1);
	      cv::resize(cv_ptr->image, downsample, Size(), 0.25, 0.25);
	      Mat matT(2,3, CV_32FC1);
	      matT.at<float>(0, 0) = matT.at<float>(1, 1) = 1.0;
	      matT.at<float>(0,1) = matT.at<float>(1,0) = 0.0;
	      matT.at<float>(0,2) = 0.0;
	      matT.at<float>(1,2) = offset;
	      
	      
	      warpAffine(downsample,downsample,matT,downsample.size());
	    }
	  }
	}
	else
	{
	  Mat downsample(rows/4, cols/4, CV_32FC1);
	  cv::resize(cv_ptr->image, downsample, Size(), 0.25, 0.25);
	  writeImage(downsample, file);
	}
      }
    }
  }
}

void writeImage(const cv::Mat &image, ofstream &file) {
  for (int r = 0; r < image.rows; r++)
  {
    for (int c = 0; c < image.cols; c++)
    {
      float pixel = image.at<float>(r,c);

      file << pixel << '\t';
    }
    file << endl;
  }
}
