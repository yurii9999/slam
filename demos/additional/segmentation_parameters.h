#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <iterator>

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

using namespace std;
using namespace rapidxml;

struct segmentation_parameters
{
    double threshold = 1.0;

    segmentation_parameters(string filename) {
        rapidxml::file<> xmlFile(filename.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        xml_node<> *th = doc.first_node("threshold");
        if (th)
            threshold = stod(th->value());
    }

    void print() {
        cout << "Segmentation parameters: " << endl;
        cout << "Threshold: " << threshold << endl;
    }
};
