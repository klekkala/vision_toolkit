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
#include <std_msgs/UInt32MultiArray.h>

#include "common/CliParser.h"

using namespace cv;
using namespace std;
using rosbag::Bag;
using boost::filesystem::path;

int main(int argc, const char *argv[])
{
  CliParser parser(argc, argv);
  parser.AddArgument("input,i", "Input rosbag file");
  parser.AddArgument("output,o", "Output file. Rosbag if .bag, video otherwise");
  parser.Parse();

  // Initialize bag and open for reading
  Bag bag;
  bag.open(parser.GetArgument("i"), rosbag::bagmode::Read);

  rosbag::View view(bag);

  // Initialize cv::bridge pointer... TODO: Verify if this is needed
  cv_bridge::CvImagePtr cv_ptr;

  // Basic Aruco detection stuffs
  std::vector<int32_t> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

  // TODO: Set up for pose estimation. (https://docs.opencv.org/4.2.0/d5/dae/tutorial_aruco_detection.html)

  // Iterate over messages in bag and convert to OpenCV image. Add to appropriate output stream
  // TODO: Figure out rosbag write...
  rosbag::Bag out;
  if (path(parser.GetArgument("o")).extension() == ".bag") {
    out.open(parser.GetArgument("o"), rosbag::bagmode::Write);
    std_msgs::UInt32MultiArray ros_arr;
    ros::Time::init();
    ros_arr.layout.dim.emplace_back(std_msgs::MultiArrayDimension());
    cout << ros_arr.layout.dim.size() << endl;

    foreach(rosbag::MessageInstance const m, view) {
      auto img_msg_ptr = m.instantiate<sensor_msgs::Image>();
      cv::Mat img = cv_bridge::toCvCopy(img_msg_ptr)->image;
      cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

      ros_arr.data.clear();
      ros_arr.data.insert(ros_arr.data.end(), markerIds.begin(), markerIds.end());
      out.write("markerIds", ros::Time::now(), ros_arr);

      markerIds.clear();
      markerCorners.clear();
      rejectedCandidates.clear();
    }
  } else {
    std::vector<cv::Mat> imgs;
    foreach(rosbag::MessageInstance const m, view) {
      auto img_msg_ptr = m.instantiate<sensor_msgs::Image>();
      cv::Mat img = cv_bridge::toCvCopy(img_msg_ptr)->image;
      cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

      if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
      }
      imgs.emplace_back(img);

      markerIds.clear();
      markerCorners.clear();
      rejectedCandidates.clear();
    }
    
    if (imgs.empty()) {
      return EXIT_FAILURE;
    }

    VideoWriter writer(parser.GetArgument("o"), VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, imgs[0].size());
    for (const auto& img : imgs) {
      writer.write(img);
    }
    writer.release();
  }

  bag.close();
  return 0;
}
