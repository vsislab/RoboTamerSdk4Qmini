#ifndef JOYSTICK_READER_H
#define JOYSTICK_READER_H
#include <iostream>
#include <Python.h>
#include <string>
#include <math.h>
#include <chrono>
#include <thread>
#include <sstream>
#include <json/json.h>

using namespace std;



class JoystickReader {
public:
    JoystickReader() {
        Py_Initialize();
        PyEval_InitThreads();
        PyRun_SimpleString("import sys; sys.path.append('.')");
    }

    ~JoystickReader() {
        Py_Finalize();
    }

    bool initJoystickReader() {
        PyGILState_STATE qqq=PyGILState_Ensure();
        auto pModule = PyImport_ImportModule("joystick");
        if (!pModule) {
            PyErr_Print();
            return false;
        }

        PyObject *pFunc = PyObject_GetAttrString(pModule, "init_joystick");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            PyErr_Print();
            Py_DECREF(pModule);
            return false;
        }

        PyObject_CallObject(pFunc, nullptr);

        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        PyGILState_Release(qqq);
        return true;
    }

    bool fetchJoystickData() {
        PyGILState_STATE qqq=PyGILState_Ensure();
        auto pModule = PyImport_ImportModule("joystick");
        if (!pModule) {
            PyErr_Print();
            return false;
        }

        PyObject *pFunc = PyObject_GetAttrString(pModule, "read_joystick");
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
        return parseJoystickData(jsonData);
    }

    bool parseJoystickData(const std::string &jsonData) {
        Json::CharReaderBuilder reader;
        Json::Value root;
        std::istringstream ss(jsonData);
        std::string errs;

        if (!Json::parseFromStream(reader, ss, &root, &errs)) {
            std::cerr << "JSON parse error: " << errs << std::endl;
            return false;
        }

        Axis[0] = root["LaxiX"].asFloat();
        Axis[1] = root["LaxiY"].asFloat();
        Axis[2] = root["RaxiX"].asFloat();
        Axis[3] = root["RaxiY"].asFloat();

        Hat[0] = root["hatX"].asInt();
        Hat[1] = root["hatY"].asInt();

        But[0] = root["butA"].asInt();
        But[1] = root["butB"].asInt();
        But[2] = root["butX"].asInt();
        But[3] = root["butY"].asInt();

        But[4] = root["L1"].asInt();
        But[5] = root["R1"].asInt();
        But[6] = root["L2"].asInt();
        But[7] = root["R2"].asInt();
        But[8] = root["SELECT"].asInt();
        But[9] = root["START"].asInt();

        return true;
    }

    void displayData() const {
        std::cout << "Joystick Data:" << std::endl;
        std::cout << "LAxis: (" << Axis[1] << "LY, " << Axis[2] << "RX)" << std::endl;
//        std::cout << "RAxis: (" << Axis[2] << ", " << Axis[3] << ")" << std::endl;
        std::cout << "Hat: (" << Hat[0] << ", " << Hat[1] << ")" << std::endl;
        std::cout << "Buttons: (" << But[0] << "A, " << But[1] << "B, " << But[2] << "X, " << But[3] << "Y)" << std::endl;
//        std::cout << "Trigers: (" << But[4] << ", " << But[5] << ", " << But[6] << ", " << But[7] << ")" << std::endl;
        std::cout << "SELECT: " << But[8] << std::endl;
//        std::cout << "START: " << But[9] << std::endl;
    }

    float Axis[4];
    int Hat[2];
    int But[10];
};
#endif // JOYSTICK_READER_H