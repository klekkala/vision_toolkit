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
	//vector<rosbag::Bag> bags = {rosbag::Bag(),rosbag::Bag(),rosbag::Bag(),
	//	                    rosbag::Bag(), rosbag::Bag()};
	rosbag::Bag bags;
	vector<string> filenames = {"cam1.bag", "cam2.bag", "cam3.bag",
		                    "cam4.bag", "cam5.bag"};

	vector<string> colorTopic1;
	vector<string> depthTopic1;
	vector<string> colorMeta1;
	vector<string> depthMeta1;
        colorTopic1.push_back(string("/cam_1/color/image_raw"));
        depthTopic1.push_back(string("/cam_1/depth/image_rect_raw"));
	colorMeta1.push_back(string("/cam_1/color/metadata"));
	depthMeta1.push_back(string("/cam_1/depth/metadata"));

	vector<string> colorTopic2;
        vector<string> depthTopic2;
	vector<string> colorMeta2;
        vector<string> depthMeta2;
        colorTopic2.push_back(string("/cam_2/color/image_raw"));
        depthTopic2.push_back(string("/cam_2/depth/image_rect_raw"));
	colorMeta2.push_back(string("/cam_2/color/metadata"));
        depthMeta2.push_back(string("/cam_2/depth/metadata"));

	vector<string> colorTopic3;
        vector<string> depthTopic3;
	vector<string> colorMeta3;
        vector<string> depthMeta3;
        colorTopic3.push_back(string("/cam_3/color/image_raw"));
        depthTopic3.push_back(string("/cam_3/depth/image_rect_raw"));
	colorMeta3.push_back(string("/cam_3/color/metadata"));
        depthMeta3.push_back(string("/cam_3/depth/metadata"));

	vector<string> colorTopic4;
        vector<string> depthTopic4;
	vector<string> colorMeta4;
        vector<string> depthMeta4;
        colorTopic4.push_back(string("/cam_4/color/image_raw"));
        depthTopic4.push_back(string("/cam_4/depth/image_rect_raw"));
	colorMeta4.push_back(string("/cam_4/color/metadata"));
        depthMeta4.push_back(string("/cam_4/depth/metadata"));

	vector<string> colorTopic5;
        vector<string> depthTopic5;
	vector<string> colorMeta5;
        vector<string> depthMeta5;
        colorTopic5.push_back(string("/cam_5/color/image_raw"));
        depthTopic5.push_back(string("/cam_5/depth/image_rect_raw"));
	colorMeta5.push_back(string("/cam_5/color/metadata"));
        depthMeta5.push_back(string("/cam_5/depth/metadata"));

	vector<vector<string>> colorTopics = {colorTopic1, colorTopic2,
		                              colorTopic3, colorTopic4,
					      colorTopic5};
	vector<vector<string>> depthTopics = {depthTopic1, depthTopic2,
		                              depthTopic3, depthTopic4,
					      depthTopic5};
	vector<vector<string>> colorMetas = {colorMeta1, colorMeta2,
		                             colorMeta3, colorMeta4,
					     colorMeta5};
	vector<vector<string>> depthMetas = {depthMeta1, depthMeta2,
		                             depthMeta3, depthMeta4,
					     depthMeta5};

	vector<int> allFrames(5, 0);

	// Read all cam files
	for (int j = 0; j < 5; j++) {
		bags.open(filenames[j]);
		rosbag::View view(bags, rosbag::TopicQuery(depthTopics[j]));
		foreach (rosbag::MessageInstance const m, view) {
			sensor_msgs::ImageConstPtr i = m.instantiate<sensor_msgs::Image>();
			if (i != NULL) {
				allFrames[j]++;
			}
		}
		bags.close();
	}

	cout << allFrames[0] << " " << allFrames[1] << " " << allFrames[2];
	cout << " " << allFrames[3] << " " << allFrames[4] << endl;

	// Find shortest stream length
	int minLen = *min_element(allFrames.begin(), allFrames.end());

	// Create new merged bag file using minLen value
	rosbag::Bag mergebag;
	ros::Time::init();
	mergebag.open("merged.bag", rosbag::bagmode::Write);
	for (int j = 0; j < 5; j++) {
		bags.open(filenames[j]);
		int count = 0;

		rosbag::View viewa(bags, rosbag::TopicQuery(colorTopics[j]));
		foreach (rosbag::MessageInstance const m, viewa) {
			sensor_msgs::ImageConstPtr i = m.instantiate<sensor_msgs::Image>();
			if (i != NULL && count < minLen) {
				mergebag.write(colorTopics[j][0],
						ros::Time::now(), *i);
				count++;
			}
		}

		count = 0;
		rosbag::View viewb(bags, rosbag::TopicQuery(depthTopics[j]));
		foreach (rosbag::MessageInstance const m, viewb) {
			sensor_msgs::ImageConstPtr i = m.instantiate<sensor_msgs::Image>();
			if (i != NULL && count < minLen) {
				mergebag.write(depthTopics[j][0],
						ros::Time::now(), *i);
				count++;
			}
		}
		
		count = 0;
		rosbag::View viewc(bags, rosbag::TopicQuery(colorMetas[j]));
		foreach (rosbag::MessageInstance const m, viewc) {
			sensor_msgs::ImageConstPtr i = m.instantiate<sensor_msgs::Image>();
			if (i != NULL && count < minLen) {
				mergebag.write(colorMetas[j][0],
						ros::Time::now(), *i);
			}
		}

		count = 0;
		rosbag::View viewd(bags, rosbag::TopicQuery(depthMetas[j]));
		foreach (rosbag::MessageInstance const m, viewd) {
			sensor_msgs::ImageConstPtr i = m.instantiate<sensor_msgs::Image>();
			if (i != NULL) {
				mergebag.write(depthMetas[j][0],
						ros::Time::now(), *i);
			}
		}

		bags.close();
	}
	mergebag.close();
}
