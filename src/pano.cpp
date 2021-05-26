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
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost::program_options;
using boost::filesystem::path;

static constexpr cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;

int main(int argc, const char *argv[]) {
   // Parse commandline args
   std::string in_str, out_str;

   // clang-format off
   options_description desc("Allowed options");
   desc.add_options()
      ("input,i", value<std::string>(&in_str),
       "Input location for input images")
      ("output,o", value<std::string>(&out_str),
       "Output location for stitched panorama");
   // clang-format on

   variables_map vm;
   store(parse_command_line(argc, argv, desc), vm);
   notify(vm);

   std::cout << "in: " << in_str << std::endl;
   std::cout << "out: " << out_str << std::endl;

   // Dummy code
   cv::Mat pano;
   cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode);

   // Read images
   std::vector<cv::Mat> imgs;
   path in_path = path(in_str);
   boost::filesystem::directory_iterator it{in_path};

   while (it != boost::filesystem::directory_iterator{}) {
      cv::Mat img = cv::imread((it->path()).string(), cv::IMREAD_COLOR);
      imgs.emplace_back(img);
      ++it;
   }

   std::cout << "Number of images: " << imgs.size() << std::endl;

   cv::Stitcher::Status status = stitcher->stitch(imgs, pano);

   cv::imwrite(out_str, pano);

   return 0;
}
