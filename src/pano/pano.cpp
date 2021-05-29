/* Panoramic stitching implementation
 *
 * \file src/pano/pano.cpp
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



   return 0;
}

class CliParser
{
public:
    CliParser(int argc, const char * const * argv);
    void AddArgument(string arg_names);
    void AddArgument(string arg_names, string explanation);
    void Parse();
    string& GetArgument(string arg);

private:
    bool _hasParsed = false;
    std::map<string, string&> _arg_map;
    options_description _desc;
    int _argc;
    const char * const * _argv;
};

CliParser::CliParser(int argc, const char * const * argv) : _argc(argc), _argv(argv)
{
}
void CliParser::AddArgument(string arg_names, string explanation)
{
    string in;
    _desc.add_options()
            (arg_names.c_str(), value<std::string>(&in),
             explanation.c_str());
    stringstream ss(arg_names);
    string word;
    while(getline(ss, word, ','))
    {
        _arg_map.insert(pair<string, string&>(word, in));
    }
}

void CliParser::Parse() {
    if(_hasParsed) return;
    variables_map vm;
    auto desc = _desc;
    store(parse_command_line(_argc, _argv, _desc), vm);
    notify(vm);
    _hasParsed = true;
}

string & CliParser::GetArgument(string arg)
{
    auto search_result = _arg_map.find(arg);
    if(search_result == _arg_map.end())
    {
        cout << "could not find argument!";
        return (string &) "";
    }
    return _arg_map.at(arg);
}


class ImageIO
{
public:
    std::vector<cv::Mat>& Read(string in_str);
    void Write(const cv::Mat& img, string out_path);
};

std::vector<cv::Mat> & ImageIO::Read(string in_str)
{
    std::vector<cv::Mat> imgs;
    path in_path = path(in_path);
    boost::filesystem::directory_iterator it{in_path};

    while (it != boost::filesystem::directory_iterator{}) {
        cv::Mat img = cv::imread((it->path()).string(), cv::IMREAD_COLOR);
        imgs.emplace_back(img);
        ++it;
    }
    return imgs;
}

void ImageIO::Write(const cv::Mat &img, string out_path)
{
    cv::imwrite(out_path, img);
}

class ImageImporter
{
public:
    ImageImporter(CliParser& parser);
    cv::Mat ImportImage();
private:
    CliParser& _parser;
    ImageIO _imageIO;
};
ImageImporter::ImageImporter(CliParser &parser) : _parser(parser)
{
    parser.AddArgument("input,i", "Input location for input images");
    parser.AddArgument("output,o","Output location for stitched panorama");
}

cv::Mat ImageImporter::ImportImage()
{
    _parser.Parse();
    _imageIO.Read(_parser.GetArgument("i"));
}