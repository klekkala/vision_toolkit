//
// Created by Slava Zinevich on 5/29/21.
//

#ifndef INC_3DOBJECT_TRACKING_CLIPARSER_H
#define INC_3DOBJECT_TRACKING_CLIPARSER_H


#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost::program_options;
using boost::filesystem::path;

class CliParser
{
public:
    CliParser(int argc, const char * const * argv);
    void AddArgument(string arg_names);
    void AddArgument(string arg_names, string explanation);
    void Parse();
    string GetArgument(string arg);

private:
    bool _hasParsed = false;
    std::map<string, string*> _arg_map;
    options_description _desc;
    variables_map _vm;
    int _argc;
    const char * const * _argv;
};


#endif //INC_3DOBJECT_TRACKING_CLIPARSER_H
