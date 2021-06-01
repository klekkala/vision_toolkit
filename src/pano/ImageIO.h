//
// Created by Slava Zinevich on 5/29/21.
//

#pragma once

#include "CliParser.h"
#include <opencv2/highgui.hpp>

class ImageIO {
public:
  ImageIO(CliParser &parser);
  std::vector<cv::Mat> Read(const std::string& in_str);
  void Write(const cv::Mat &img);
  std::vector<cv::Mat> ImportImages();

private:
  CliParser &_parser;
};
