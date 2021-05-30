//
// Created by Slava Zinevich on 5/29/21.
//

#ifndef INC_3DOBJECT_TRACKING_IMAGEIO_H
#define INC_3DOBJECT_TRACKING_IMAGEIO_H
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include "CliParser.h"

using namespace std;
using namespace boost::program_options;
using boost::filesystem::path;

class ImageIO
{
public:
    std::vector<cv::Mat>& Read(string in_str);
    void Write(const cv::Mat& img);
    ImageIO(CliParser& parser);
    std::vector<cv::Mat>& ImportImages();
private:
    CliParser& _parser;
};

#endif //INC_3DOBJECT_TRACKING_IMAGEIO_H
