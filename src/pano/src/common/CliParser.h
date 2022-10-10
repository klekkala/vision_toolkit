//
// Created by Slava Zinevich on 5/29/21.
// Edited by Mihai Bibireata on 5/30/21.
//

#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

class CliParser
{
public:
    CliParser(int argc, const char * const * argv);
    void AddArgument(const std::string& arg_names);
    void AddArgument(const std::string& arg_names, const std::string& explanation);
    void Parse();
    std::string GetArgument(const std::string& arg);

private:
    bool _hasParsed = false;
    std::map<std::string, std::string*> _arg_map;
    boost::program_options::options_description _desc;
    boost::program_options::variables_map _vm;
    int _argc;
    const char * const * _argv;
};
