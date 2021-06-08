/* Aruco pose estimation implementation
 *
 * \file src/aruco/aruco_pose.cpp
 * \author Mihai Bibireata (bibireat@usc.edu)
 * \author Slava Zinevich (zinevich@usc.edu)
 * \date May 28 2021
 */

#include <iostream>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>

#include "common/CliParser.h"

using namespace cv;
using namespace std;
using rosbag::Bag;

int main(int argc, const char *argv[])
{
  CliParser parser(argc, argv);
  parser.AddArgument("input,i", "Input rosbag file");
  parser.AddArgument("output,o", "Output video file");
  parser.Parse();

  // Initialize bag and open for reading
  Bag bag;
  bag.open(parser.GetArgument("i"), rosbag::bagmode::Read);

  rosbag::View view(bag);
  cout << view.size() << endl;

  // Initialize cv::bridge pointer... TODO: Verify if this is needed
  cv_bridge::CvImagePtr cv_ptr;

  
  // Iterate over messages in bag and convert to OpenCV image. Add to video writer
  //VideoWriter writer(parser.GetArgument("o"), VideoWriter::fourcc('M', 'J', 'P', 'G'), 15);
  //VideoWriter writer();
  // TODO: This is temp hack code to get some aruco markers to show. Don't need video shenanigans 
  std::vector<cv::Mat> imgs;
  foreach(rosbag::MessageInstance const m, view) {
    auto img_msg_ptr = m.instantiate<sensor_msgs::Image>();
    cv::Mat img = cv_bridge::toCvCopy(img_msg_ptr)->image;
    imgs.emplace_back(img);
  }
  
  if (imgs.empty()) {
    return EXIT_FAILURE;
  }

  VideoWriter writer(parser.GetArgument("o"), VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, imgs[0].size());
  for (const auto& img : imgs) {
    writer.write(img);
  }
  writer.release();

  bag.close();
  return 0;
}
