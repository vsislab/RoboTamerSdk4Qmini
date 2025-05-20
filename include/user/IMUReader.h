#include <iostream>
#include <Python.h>
#include <string>
#include <chrono>
#include <thread>
#include <sstream>
#include <json/json.h>

using namespace std;

class IMUReader {
public:
    IMUReader() {
        Py_Initialize();
        PyEval_InitThreads();
        PyRun_SimpleString("import sys; sys.path.append('.')");
    }

    ~IMUReader() {
        Py_Finalize();
    }

    bool fetchIMUData() {
        PyGILState_STATE qqq=PyGILState_Ensure();
        auto pModule = PyImport_ImportModule("imu_interface");
        if (!pModule) {
            PyErr_Print();
            return false;
        }

        PyObject *pFunc = PyObject_GetAttrString(pModule, "get_imu_data");

        if (!pFunc || !PyCallable_Check(pFunc)) {
            PyErr_Print();
            Py_DECREF(pModule);
            return false;
        }

        PyObject *pValue = PyObject_CallObject(pFunc, nullptr);

        if (!pValue) {
            PyErr_Print();
            Py_DECREF(pFunc);
            Py_DECREF(pModule);
            return false;
        }

        std::string jsonData = PyUnicode_AsUTF8(pValue);
        Py_DECREF(pValue);
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        PyGILState_Release(qqq);
        return parseIMUData(jsonData);
    }

    void displayData() const {
        std::cout << "IMU Data:" << std::endl;
        std::cout << "Accelerometer: (" << Accelerometer_X << ", " << Accelerometer_Y << ", " << Accelerometer_Z << ")" << std::endl;
        std::cout << "RollSpeed: " << RollSpeed << ", PitchSpeed: " << PitchSpeed << ", HeadingSpeed: " << HeadingSpeed << std::endl;
        std::cout << "Roll: " << Roll << ", Pitch: " << Pitch << ", Heading: " << Heading << std::endl;
        std::cout << "Quaternion: (" << qw << ", " << qx << ", " << qy << ", " << qz << ")" << std::endl;
    }

    bool parseIMUData(const std::string &jsonData) {
        Json::CharReaderBuilder reader;
        Json::Value root;
        std::istringstream ss(jsonData);
        std::string errs;

        if (!Json::parseFromStream(reader, ss, &root, &errs)) {
            std::cerr << "JSON parse error: " << errs << std::endl;
            return false;
        }

        Accelerometer_X = root["Accelerometer_X"].asFloat();
        Accelerometer_Y = root["Accelerometer_Y"].asFloat();
        Accelerometer_Z = root["Accelerometer_Z"].asFloat();
        RollSpeed = root["RollSpeed"].asFloat();
        PitchSpeed = root["PitchSpeed"].asFloat();
        HeadingSpeed = root["HeadingSpeed"].asFloat();
        Roll = root["Roll"].asFloat();
        Pitch = root["Pitch"].asFloat();
        Heading = root["Heading"].asFloat();
        qw = root["qw"].asFloat();
        qx = root["qx"].asFloat();
        qy = root["qy"].asFloat();
        qz = root["qz"].asFloat();

        return true;
    }

    float Accelerometer_X, Accelerometer_Y, Accelerometer_Z;
    float RollSpeed, PitchSpeed, HeadingSpeed;
    float Roll, Pitch, Heading;
    float qw, qx, qy, qz;
};