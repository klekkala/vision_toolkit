/* Panoramic stitching implementation
 *
 * \file src/pano/pano.cpp
 * \author Mihai Bibireata (bibireat@usc.edu)
 * \author Slava Zinevich (zinevich@usc.edu)
 * \date May 26 2021
 */

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

#include "common/CliParser.h"
#include "common/ImageIO.h"

using namespace std;
using namespace boost::program_options;
using boost::filesystem::path;

static constexpr cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;

int main(int argc, const char *argv[]) {
  CliParser parser(argc, argv);
  parser.AddArgument("input,i", "Input location for input images");
  parser.AddArgument("output,o", "Output location for stitched panorama");
  parser.Parse();

  ImageIO image_io;
  auto imgs = image_io.Read(parser.GetArgument("i"));

  // Dummy code
  cv::Mat pano;
  cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode);
  std::cout << "Number of images: " << imgs.size() << std::endl;

  cv::Stitcher::Status status = stitcher->stitch(imgs, pano);
  image_io.Write(parser.GetArgument("o"), pano);

  return 0;
}
