/* Aruco pose estimation implementation
 *
 * \file src/aruco/aruco_pose.cpp
 * \author Mihai Bibireata (bibireat@usc.edu)
 * \author Slava Zinevich (zinevich@usc.edu)
 * \date May 28 2021
 */

#include <iostream>
#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking.hpp>
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

typedef struct MarkerData {
  std::vector<cv::Point2f> corners;
  cv::Vec3d rvec, tvec;
} MarkerData;

typedef struct ObsData {
  std::unordered_map<int32_t, struct MarkerData*> id_to_data;
  std::vector<opencv_aruco_2d_pts_t> rejected_candidates;
} ObsData;

typedef std::unordered_map<int32_t, std::vector<size_t>> marker_to_frame_t;
typedef std::unordered_map<size_t, struct ObsData*> frame_to_obs_t;

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
                 frame_to_obs_t& frame_to_obs,
                 marker_to_frame_t& marker_to_frame,
                 cv::Mat& cam_mat,
                 cv::Mat& dst_mat,
                 std::vector<cv::Mat>& imgs) {
  cout << "Detecting aruco markers..." << endl;
  rosbag::View view(bag);
  std::vector<int32_t> ids;
  opencv_aruco_2d_pts_t corners, rejected;

  size_t frame_id = 0;
  foreach(rosbag::MessageInstance const m, view) {
    struct ObsData* obs_data = new ObsData();
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

    // Create tracker
    /*
    Ptr<Tracker> tracker = TrackerKCF::create();
    TrackerSamplerPF::Params PFparams;
    for (const auto& marker_id : marker_ids[0]) {
      cout << "ID: " << marker_id << endl;
    }
    auto rect = RectBBOXFromPts(marker_corners[0][0]);
    Ptr<TrackerSamplerAlgorithm> sampler = new TrackerSamplerPF(rect, PFparams);
    //if (!tracker->sampler->addTrackSamplerAlgorithm(
    */

    // Save pertinent data
    imgs.emplace_back(img);
    for (int i = 0; i < rvecs.size(); ++i) {
      struct MarkerData* marker_data = new MarkerData();
      marker_data->corners = corners[i];
      marker_data->tvec = tvecs[i];
      marker_data->rvec = rvecs[i];
      obs_data->id_to_data.emplace(ids[i], marker_data);

      if (marker_to_frame.find(ids[i]) != marker_to_frame.end()) {
        marker_to_frame.at(ids[i]).emplace_back(frame_id);
      } else {
        marker_to_frame.emplace(ids[i], std::vector<size_t>({ frame_id }));
      }
    } 
    obs_data->rejected_candidates.emplace_back(rejected);
    frame_to_obs.emplace(frame_id, obs_data);

    // Clear vectors for next iteration
    ids.clear();
    corners.clear();
    rejected.clear();

    ++frame_id;
  }
}

/** Generate a cv::Rect bounding box for aruco marker corners
 *
 * \param[in] pts   Points to generate bounding box for
 *
 * \returns         Rect object bounding box
 */
cv::Rect RectBBOXFromPts(const std::vector<cv::Point2f>& pts) {
  float tl_x=pts[0].x, tl_y=pts[0].y, br_x=0., br_y=0., w, h;
  for (const auto& pt : pts) {
    tl_x = min(pt.x, tl_x);
    tl_y = min(pt.y, tl_y);
    br_x = max(pt.x, br_x);
    br_y = max(pt.y, br_y);
  }
  return cv::Rect(tl_x, tl_y, br_x - tl_x, br_y - tl_y);
}

void InterpolateMarkers(const frame_to_obs_t& frame_to_obs,
                        const marker_to_frame_t& marker_to_frame,
                        const cv::Mat& cam_mat,
                        const cv::Mat& dst_mat,
                        std::vector<cv::Mat>& imgs) {
  for (const auto& marker_and_frame : marker_to_frame) {
    auto& marker_id = marker_and_frame.first;
    auto& frames = marker_and_frame.second;

    size_t gap = 0;
    for (size_t i = 1; i < frames.size(); ++i) {
      gap = frames[i] - frames[i - 1]; 
      if (gap == 2) continue;
      auto& t_init = frame_to_obs.at(frames[i - 1])->id_to_data.at(marker_id)->tvec;
      auto& r_init = frame_to_obs.at(frames[i - 1])->id_to_data.at(marker_id)->rvec;
      auto& t_final = frame_to_obs.at(frames[i])->id_to_data.at(marker_id)->tvec;
      auto& r_final = frame_to_obs.at(frames[i])->id_to_data.at(marker_id)->rvec;

      auto d_trans = t_final - t_init;
      auto d_rot = r_final - r_init;
      cv::divide(d_trans, gap, d_trans);
      cv::divide(d_rot, gap, d_rot);

      // Debug prints
      /*
      cout << "init frame: " << frames[i - 1] << ", final frame: " << frames[i] << endl;
      cout << "t init:  " << t_init << endl;
      cout << "r init:  " << r_init << endl;
      cout << "t final: " << t_final << endl;
      cout << "r final: " << r_final << endl;
      cout << "d trans: " << d_trans << endl;
      cout << "d rot:   " << d_rot << endl;
      */

      // Draw intermediate poses
      for (size_t j = 1; j <= gap; ++j) {
        auto target_frame = frames[i - 1] + j;
        cv::Vec3d target_t, target_r;
        cv::multiply(d_trans, j, target_t);
        cv::multiply(d_rot, j, target_r);
        target_t += t_init;
        target_r += r_init;
        //cv::aruco::drawAxis(imgs[target_frame], cam_mat, dst_mat, target_r, target_t, 0.1);
        cv::aruco::drawAxis(imgs[target_frame], cam_mat, dst_mat, r_init, target_t, 0.1);
        
      }
    }
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
  frame_to_obs_t frame_to_obs;
  marker_to_frame_t marker_to_frame;
  std::vector<cv::Mat> imgs;

  DetectAruco(bag, parameters, dictionary, frame_to_obs, marker_to_frame, cam_mat, dst_mat, imgs);

  cout << "Number of markers detected: " << marker_to_frame.size() << endl;

  InterpolateMarkers(frame_to_obs, marker_to_frame, cam_mat, dst_mat, imgs);
  
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
