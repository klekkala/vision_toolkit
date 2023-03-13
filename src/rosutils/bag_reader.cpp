#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#define foreach BOOST_FOREACH
using namespace std;

int main() {
	// Initialize all variables
	rosbag::Bag bag1, bag2, bag3, bag4, bag5;

	vector<string> colorTopic1;
	vector<string> depthTopic1;
        colorTopic1.push_back(string("/cam_1/color/image_raw"));
        depthTopic1.push_back(string("/cam_1/depth/image_rect_raw"));

	vector<string> colorTopic2;
        vector<string> depthTopic2;
        colorTopic2.push_back(string("/cam_2/color/image_raw"));
        depthTopic2.push_back(string("/cam_2/depth/image_rect_raw"));

	vector<string> colorTopic3;
        vector<string> depthTopic3;
        colorTopic3.push_back(string("/cam_3/color/image_raw"));
        depthTopic3.push_back(string("/cam_3/depth/image_rect_raw"));

	vector<string> colorTopic4;
        vector<string> depthTopic4;
        colorTopic4.push_back(string("/cam_4/color/image_raw"));
        depthTopic4.push_back(string("/cam_4/depth/image_rect_raw"));

	vector<string> colorTopic5;
        vector<string> depthTopic5;
        colorTopic5.push_back(string("/cam_5/color/image_raw"));
        depthTopic5.push_back(string("/cam_5/depth/image_rect_raw"));

	vector<int> allFrames(5, 0);

	// Read cam1 file
	bag1.open("cam1.bag");
	rosbag::View view1(bag1, rosbag::TopicQuery(depthTopic1));
	foreach (rosbag::MessageInstance const m, view1) {
		sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
		if (i != NULL) {
			allFrames[0]++;
		}
	}
	bag1.close();

	// Read cam2 file
        bag2.open("cam2.bag");
	rosbag::View view2(bag2, rosbag::TopicQuery(depthTopic2));
        foreach (rosbag::MessageInstance const m, view2) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        allFrames[1]++;
                }
        }
        bag2.close();

	// Read cam3 file
        bag3.open("cam3.bag");
	rosbag::View view3(bag3, rosbag::TopicQuery(depthTopic3));
        foreach (rosbag::MessageInstance const m, view3) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        allFrames[2]++;
                }
        }
        bag3.close();

	// Read cam4 file
        bag4.open("cam4.bag");
	rosbag::View view4(bag4, rosbag::TopicQuery(depthTopic4));
        foreach (rosbag::MessageInstance const m, view4) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        allFrames[3]++;
                }
        }
        bag4.close();

	// Read cam5 file
        bag5.open("cam5.bag");
	rosbag::View view5(bag5, rosbag::TopicQuery(depthTopic5));
        foreach (rosbag::MessageInstance const m, view5) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        allFrames[4]++;
                }
        }
        bag5.close();

	cout << allFrames[0] << " " << allFrames[1] << " " << allFrames[2];
	cout << " " << allFrames[3] << " " << allFrames[4] << endl;

	// Find shortest stream length
	int minLen = *min_element(allFrames.begin(), allFrames.end());

	// Remove frames from bags
	//rosbag::Bag newbag1, newbag2, newbag3, newbag4, newbag5;
	rosbag::Bag mergebag;
	ros::Time::init();

	// Write to newcam1 file
	bag1.open("cam1.bag");
	//newbag1.open("newcam1.bag", rosbag::bagmode::Write);
	mergebag.open("merged.bag", rosbag::bagmode::Write);
	int count = 0;
        rosbag::View view1a(bag1, rosbag::TopicQuery(colorTopic1));
        foreach (rosbag::MessageInstance const m, view1a) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_1/color/image_raw",
					ros::Time::now(), *i);
			count++;
                }
        }
	count = 0;
	rosbag::View view1b(bag1, rosbag::TopicQuery(depthTopic1));
        foreach (rosbag::MessageInstance const m, view1b) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_1/depth/image_rect_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
	count = 0;
	vector<string> metacTopic1;
	metacTopic1.push_back(string("/cam_1/color/metadata"));
	vector<string> metadTopic1;
	metadTopic1.push_back(string("/cam_1/depth/metadata"));
	rosbag::View view1c(bag1, rosbag::TopicQuery(metacTopic1));
        foreach (rosbag::MessageInstance const m, view1c) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_1/color/metadata",
                                        ros::Time::now(), *i);
                }
        }
	rosbag::View view1d(bag1, rosbag::TopicQuery(metadTopic1));
        foreach (rosbag::MessageInstance const m, view1d) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_1/depth/metadata",
                                        ros::Time::now(), *i);
                }
        }
        bag1.close();
	//newbag1.close();

	// Write to newcam2 file
        bag2.open("cam2.bag");
        //newbag2.open("newcam2.bag", rosbag::bagmode::Write);
        rosbag::View view2a(bag2, rosbag::TopicQuery(colorTopic2));
        foreach (rosbag::MessageInstance const m, view2a) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_2/color/image_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
        count = 0;
        rosbag::View view2b(bag2, rosbag::TopicQuery(depthTopic2));
        foreach (rosbag::MessageInstance const m, view2b) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_2/depth/image_rect_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
        count = 0;
        vector<string> metacTopic2;
        metacTopic2.push_back(string("/cam_2/color/metadata"));
        vector<string> metadTopic2;
        metadTopic2.push_back(string("/cam_2/depth/metadata"));
        rosbag::View view2c(bag2, rosbag::TopicQuery(metacTopic2));
        foreach (rosbag::MessageInstance const m, view2c) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_2/color/metadata",
                                        ros::Time::now(), *i);
                }
        }
        rosbag::View view2d(bag2, rosbag::TopicQuery(metadTopic2));
        foreach (rosbag::MessageInstance const m, view2d) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_2/depth/metadata",
                                        ros::Time::now(), *i);
                }
        }
        bag2.close();
        //newbag2.close();

	// Write to newcam3 file
        bag3.open("cam3.bag");
        //newbag3.open("newcam3.bag", rosbag::bagmode::Write);
        rosbag::View view3a(bag3, rosbag::TopicQuery(colorTopic3));
        foreach (rosbag::MessageInstance const m, view3a) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_3/color/image_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
        count = 0;
        rosbag::View view3b(bag3, rosbag::TopicQuery(depthTopic3));
        foreach (rosbag::MessageInstance const m, view3b) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_3/depth/image_rect_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
        count = 0;
        vector<string> metacTopic3;
        metacTopic3.push_back(string("/cam_3/color/metadata"));
        vector<string> metadTopic3;
        metadTopic3.push_back(string("/cam_3/depth/metadata"));
        rosbag::View view3c(bag3, rosbag::TopicQuery(metacTopic3));
        foreach (rosbag::MessageInstance const m, view3c) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_3/color/metadata",
                                        ros::Time::now(), *i);
                }
        }
        rosbag::View view3d(bag3, rosbag::TopicQuery(metadTopic3));
        foreach (rosbag::MessageInstance const m, view3d) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_3/depth/metadata",
                                        ros::Time::now(), *i);
                }
        }
        bag3.close();
        //newbag3.close();

	// Write to newcam4 file
        bag4.open("cam4.bag");
        //newbag4.open("newcam4.bag", rosbag::bagmode::Write);
        rosbag::View view4a(bag4, rosbag::TopicQuery(colorTopic4));
        foreach (rosbag::MessageInstance const m, view4a) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_4/color/image_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
        count = 0;
        rosbag::View view4b(bag4, rosbag::TopicQuery(depthTopic4));
        foreach (rosbag::MessageInstance const m, view4b) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_4/depth/image_rect_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
        count = 0;
        vector<string> metacTopic4;
        metacTopic4.push_back(string("/cam_4/color/metadata"));
        vector<string> metadTopic4;
        metadTopic4.push_back(string("/cam_4/depth/metadata"));
        rosbag::View view4c(bag4, rosbag::TopicQuery(metacTopic4));
        foreach (rosbag::MessageInstance const m, view4c) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
			mergebag.write("/cam_4/color/metadata",
                                        ros::Time::now(), *i);
                }
        }
        rosbag::View view4d(bag4, rosbag::TopicQuery(metadTopic4));
        foreach (rosbag::MessageInstance const m, view4d) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_4/depth/metadata",
                                        ros::Time::now(), *i);
                }
        }
        bag4.close();
        //newbag4.close();

	// Write to newcam5 file
        bag5.open("cam5.bag");
        //newbag5.open("newcam5.bag", rosbag::bagmode::Write);
        rosbag::View view5a(bag5, rosbag::TopicQuery(colorTopic5));
        foreach (rosbag::MessageInstance const m, view5a) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_5/color/image_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
        count = 0;
        rosbag::View view5b(bag5, rosbag::TopicQuery(depthTopic5));
        foreach (rosbag::MessageInstance const m, view5b) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL && count < minLen) {
                        mergebag.write("/cam_5/depth/image_rect_raw",
                                        ros::Time::now(), *i);
                        count++;
                }
        }
        count = 0;
        vector<string> metacTopic5;
        metacTopic5.push_back(string("/cam_5/color/metadata"));
        vector<string> metadTopic5;
        metadTopic5.push_back(string("/cam_5/depth/metadata"));
        rosbag::View view5c(bag5, rosbag::TopicQuery(metacTopic5));
        foreach (rosbag::MessageInstance const m, view5c) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_5/color/metadata",
                                        ros::Time::now(), *i);
                }
        }
        rosbag::View view5d(bag5, rosbag::TopicQuery(metadTopic5));
        foreach (rosbag::MessageInstance const m, view5d) {
                sensor_msgs::ImageConstPtr i=m.instantiate<sensor_msgs::Image>();
                if (i != NULL) {
                        mergebag.write("/cam_5/depth/metadata",
                                        ros::Time::now(), *i);
                }
        }
        bag5.close();
        //newbag5.close();
	mergebag.close();
}
