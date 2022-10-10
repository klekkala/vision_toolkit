//
// Created by Slava Zinevich on 5/29/21.
//

#pragma once

#include "CliParser.h"
#include <opencv2/highgui.hpp>

class ImageIO {
public:
  ImageIO() {};
  std::vector<cv::Mat> Read(const std::string& in_str);
  void Write(const std::string &out_str, const cv::Mat &img);
};
