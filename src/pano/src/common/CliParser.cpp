//
// Created by Slava Zinevich on 5/29/21.
//

#include "CliParser.h"

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost::program_options;
using boost::filesystem::path;

CliParser::CliParser(int argc, const char * const * argv) : _argc(argc), _argv(argv)
{
}
void CliParser::AddArgument(const string& arg_names, const string& explanation)
{
    string* in = new string("hello");
    _desc.add_options()
            (arg_names.c_str(), value<std::string>(in),
             explanation.c_str());
    stringstream ss(arg_names);
    string word;
    while(getline(ss, word, ','))
    {
        _arg_map.insert(pair<string, string*>(word,in));
    }
}

void CliParser::Parse() {
    if(_hasParsed) return;
    store(parse_command_line(_argc, _argv, _desc), _vm);
    notify(_vm);
    _hasParsed = true;
}

string CliParser::GetArgument(const string& arg)
{
    auto search_result = _arg_map.find(arg);
    if(search_result == _arg_map.end())
    {
        cout << "could not find argument!";
        return (string &) "";
    }
    return *_arg_map.at(arg);
}

