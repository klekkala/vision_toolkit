/* Panoramic stitching implementation
 *
 * \file src/pano.cpp
 * \author Mihai Bibireata (bibireat@usc.edu)
 * \author Slava Zinevich (zinevich@usc.edu)
 * \date May 26 2021
 */

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <boost/program_options.hpp>

using namespace std;
using namespace boost::program_options;

static constexpr cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;

int main(int argc, const char *argv[]) {
   // Parse commandline args
   std::string in_path, out_path;

   // clang-format off
   options_description desc("Allowed options");
   desc.add_options()
      ("input,i", value<std::string>(&in_path),
       "Input location for input images")
      ("output,o", value<std::string>(&out_path),
       "Output location for stitched panorama");
   // clang-format on

   variables_map vm;
   store(parse_command_line(argc, argv, desc), vm);
   notify(vm);

   // Dummy code
   cv::Mat pano;
   cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode);

   std::vector<cv::Mat> imgs;
   cv::Stitcher::Status status = stitcher->stitch(imgs, pano);

   return 0;
}
