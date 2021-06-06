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
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "common/CliParser.h"

using namespace cv;
using namespace std;
using rosbag::Bag;

int main(int argc, const char *argv[])
{
  CliParser parser(argc, argv);
  parser.AddArgument("input,i", "Input mp4 file");
  parser.Parse();

  Bag bag;
  bag.open(parser.GetArgument("i"), rosbag::bagmode::Read);

  rosbag::View view(bag);

  foreach(rosbag::MessageInstance const m, view) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s!=NULL) {
      cout << s->data << endl;
    }
  }

   /*
   VideoCapture cap(parser.GetArgument("i"));
   cout << parser.GetArgument("i") << endl;

  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
	
  while(1){

    Mat frame;
    // Capture frame-by-frame
    cap >> frame;
 
    // If the frame is empty, break immediately
    if (frame.empty())
      break;

    // Display the resulting frame
    imshow( "Frame", frame );

    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }
 
  // When everything done, release the video capture object
  cap.release();

  // Closes all the frames
  destroyAllWindows();
  */

  bag.close();
  return 0;
}
