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
#include <opencv2/stitching/detail/exposure_compensate.hpp>

#include "common/CliParser.h"
#include "common/ImageIO.h"

using namespace std;
using namespace boost::program_options;
using boost::filesystem::path;

static constexpr cv::Stitcher::Mode modePAN = cv::Stitcher::PANORAMA;
static constexpr cv::Stitcher::Mode modeSCAN = cv::Stitcher::SCANS;

int main(int argc, const char *argv[]) {
  CliParser parser(argc, argv);
  parser.AddArgument("input,i", "Input location for input images");
  parser.AddArgument("output,o", "Output location for stitched panorama");
  parser.Parse();

  ImageIO image_io;
  auto imgs = image_io.Read(parser.GetArgument("i"));

  // Dummy code
  cv::Mat pano;
  cv::Ptr<cv::Stitcher> stitcherPAN = cv::Stitcher::create(modePAN);
  std::cout << "Number of images: " << imgs.size() << std::endl;

  // Panorama parameters
  //stitcherPAN->setPanoConfidenceThresh(0.2);
  //stitcherPAN->setWaveCorrection(true);
  //stitcherPAN->setBundleAdjuster(cv::makePtr<cv::detail::BundleAdjusterRay>());
  //stitcherPAN->setWarper(cv::makePtr<cv::SphericalWarper>());
  //stitcherPAN->setExposureCompensator(cv::makePtr<cv::detail::BlocksGainCompensator>());
  //stitcherPAN->setSeamFinder(cv::makePtr<cv::detail::VoronoiSeamFinder>());
  //stitcherPAN->setBlender(cv::makePtr<cv::detail::MultiBandBlender>());

  cv::Stitcher::Status status = stitcherPAN->stitch(imgs, pano);

  // Scan method
  cv::Mat scan;
  cv::Ptr<cv::Stitcher> stitcherSCAN = cv::Stitcher::create(modeSCAN);

  // Scan parameters
  stitcherSCAN->setPanoConfidenceThresh(1.0);
  //stitcherSCAN->setWaveCorrection(true);
  //stitcherSCAN->setBundleAdjuster(cv::makePtr<cv::detail::BundleAdjusterRay>());
  //stitcherSCAN->setWarper(cv::makePtr<cv::SphericalWarper>());
  //stitcherSCAN->setExposureCompensator(cv::makePtr<cv::detail::BlocksGainCompensator>());
  stitcherSCAN->setSeamFinder(cv::makePtr<cv::detail::VoronoiSeamFinder>());
  stitcherSCAN->setBlender(cv::makePtr<cv::detail::MultiBandBlender>());

  cv::Stitcher::Status statusSCAN = stitcherSCAN->stitch(imgs, scan);

  if (status == cv::Stitcher::OK) {
    image_io.Write(parser.GetArgument("o"), pano);
  }
  else if (statusSCAN == cv::Stitcher::OK) {
    image_io.Write(parser.GetArgument("o"), scan);
    std::cout << "Panorama status: " << status << std::endl;
    std::cout << "Scan stitcher" << std::endl;
  }

  return 0;
}
