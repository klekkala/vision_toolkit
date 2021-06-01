//
// Created by Slava Zinevich on 5/29/21.
//

#include "ImageIO.h"
#include <iostream>

#include "CliParser.h"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

using namespace std;
using namespace boost::program_options;
using boost::filesystem::path;

std::vector<cv::Mat> ImageIO::Read(const string &in_str) {
  std::vector<cv::Mat> images;
  path in_path = path(in_str);
  boost::filesystem::directory_iterator it{in_path};

  while (it != boost::filesystem::directory_iterator{}) {
    cv::Mat img = cv::imread((it->path()).string(), cv::IMREAD_COLOR);
    if (img.data != NULL)
      images.emplace_back(img);
    ++it;
  }
  return images;
}

void ImageIO::Write(const string &out_str, const cv::Mat &img) {
  cv::imwrite(out_str, img);
}
