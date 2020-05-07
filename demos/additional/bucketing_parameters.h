#pragma once

#include <string>

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

#include <egomotionestimation.h>

using namespace std;
using namespace rapidxml;

struct bucketing_parameters
{
    int amount_per_cell = 2;
    int bucket_width = 50;
    int bucket_height = 50;

    bucketing_parameters(string filename) {
        rapidxml::file<> xmlFile(filename.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        xml_node<> *apc = doc.first_node("amount");
        if (apc)
            amount_per_cell = stoi(apc->value());

        xml_node<> *bw = doc.first_node("bucket_width");
        if (bw)
            bucket_width = stoi(bw->value());

        xml_node<> *bh = doc.first_node("bucket_height");
        if (bh)
            bucket_height = stoi(bh->value());
    }

    void print() {
        cout << "Bucketing parameters: " << endl;
        cout << "Amount per cell: " << amount_per_cell << endl;
        cout << "Bucket widht: " << bucket_width << endl;
        cout << "Bucket height: " << bucket_height << endl;
    }
};
