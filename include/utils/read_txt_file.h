//
// Created by cyy on 22-4-16.
//

#ifndef ALIENGO_INTERFACE_READ_FILE_H
#define ALIENGO_INTERFACE_READ_FILE_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cassert>
#include <sstream>

using namespace std;

class ReadTxtFile {
public:
    ReadTxtFile() = default;

    static void get_data_to_vector(vector<vector<float>> &data) {
        ifstream infile;
        infile.open("data/gait_data.txt");
        assert(infile.is_open());
        vector<float> temp;
        string s;
        while (getline(infile, s)) {
            istringstream is(s);
            float d;
            while (!is.eof()) {
                is >> d;
                temp.push_back(d);
            }
            temp.pop_back();
            data.push_back(temp);
            temp.clear();
            s.clear();
        }
        data.pop_back();
        infile.close();
    }
};


#endif //ALIENGO_INTERFACE_READ_FILE_H
