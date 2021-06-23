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

typedef std::vector<std::vector<cv::Point2f>> opencv_aruco_2d_pts_t;

/** Detects aruco markers
 *
 * \param[in] bag                   ROSbag to read from
 * \param[in] parameters            Aruco detector parameters
 * \param[in] dictionary            Marker dictionary to use
 * \param[out] marker_ids           Marker IDs
 * \param[out] marker_corners       Marker corners
 * \param[out] rejected_candidates  Rejected candidate corners
 * \param[out] imgs                 Output images with aruco markers drawn
 */
void DetectAruco(const Bag& bag, 
                 const cv::Ptr<cv::aruco::DetectorParameters>& parameters,
                 const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                 std::vector<std::vector<int32_t>>& marker_ids, 
                 std::vector<opencv_aruco_2d_pts_t>& marker_corners, 
                 std::vector<opencv_aruco_2d_pts_t>& rejected_candidates, 
                 cv::Mat& cam_mat,
                 cv::Mat& dst_mat,
                 std::vector<cv::Mat>& imgs) {
  cout << "Detecting aruco markers..." << endl;
  rosbag::View view(bag);
  std::vector<int32_t> ids;
  opencv_aruco_2d_pts_t corners, rejected;

  foreach(rosbag::MessageInstance const m, view) {
    auto img_msg_ptr = m.instantiate<sensor_msgs::Image>();
    cv::Mat img = cv_bridge::toCvCopy(img_msg_ptr)->image;
    cv::aruco::detectMarkers(img, dictionary, corners, ids, parameters, rejected,
                             cam_mat, dst_mat);

    // Debug: Draw all corner circles
    for (const auto& marker_corners : corners) {
      for (const auto& corner : marker_corners) {
        cv::circle(img, corner, 7, cv::Scalar(0, 255, 0), -1);
      }
    }

    // Debug: Draw rejected candidates
    for (const auto& marker_corners : rejected) {
      for (const auto& corner : marker_corners) {
        cv::circle(img, corner, 7, cv::Scalar(0, 0, 255), -1);
      }
    }

    // Draw aruco markers
    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(img, corners, ids);
    }

    // Draw marker poses
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, 0.1, cam_mat, dst_mat,
                                         rvecs, tvecs);
    for (int i = 0; i < rvecs.size(); ++i) {
      cv::aruco::drawAxis(img, cam_mat, dst_mat, rvecs[i], tvecs[i], 0.1);
    }

    imgs.emplace_back(img);
    marker_ids.emplace_back(ids);
    marker_corners.emplace_back(corners);
    rejected_candidates.emplace_back(rejected);

    ids.clear();
    corners.clear();
    rejected.clear();
  }
}

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

  // TODO: Set up for pose estimation. (https://docs.opencv.org/4.2.0/d5/dae/tutorial_aruco_detection.html)
  // Debug, for now use hard-coded calibration
  // ---------------------------------------------------------------
  cv::Mat cam_mat(cv::Size(3, 3), CV_64FC1);
  cv::Mat dst_mat(cv::Size(5, 1), CV_64FC1);
  cam_mat.at<double>(0, 0) = 998.5440063477;
  cam_mat.at<double>(0, 1) = 0.;
  cam_mat.at<double>(0, 2) = 632.0083007813;
  cam_mat.at<double>(1, 0) = 0.;
  cam_mat.at<double>(1, 1) = 1003.3219604492;
  cam_mat.at<double>(1, 2) = 346.2489318848;
  cam_mat.at<double>(2, 0) = 0.;
  cam_mat.at<double>(2, 1) = 0.;
  cam_mat.at<double>(2, 2) = 1.;

  dst_mat.at<double>(0, 0) = 0.379692703;
  dst_mat.at<double>(0, 1) = -2.033090114;
  dst_mat.at<double>(0, 2) = 0.;
  dst_mat.at<double>(0, 3) = 0.;
  dst_mat.at<double>(0, 4) = 3.4598429203;
  // ---------------------------------------------------------------

  cout << "Calibration: \n" << cam_mat << endl;
  cout << "Distortion : \n" << dst_mat << endl;

  // TODO: Figure out rosbag write...
  // Basic Aruco detection stuffs
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<std::vector<int32_t>> marker_ids;
  std::vector<opencv_aruco_2d_pts_t> marker_corners, rejected_candidates;
  std::vector<cv::Mat> imgs;

  DetectAruco(bag, parameters, dictionary, marker_ids, marker_corners,
              rejected_candidates, cam_mat, dst_mat, imgs);
  
  if (imgs.empty()) {
    return EXIT_FAILURE;
  }

  cout << "Writing video out..." << endl;
  VideoWriter writer(parser.GetArgument("o"), VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, imgs[0].size());
  for (const auto& img : imgs) {
    writer.write(img);
  }
  writer.release();

  bag.close();
  return 0;
}
