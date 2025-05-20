#include <unistd.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <array>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <fstream>
#include <iomanip>

struct SerialGroup {
    const char *port;
    std::vector<int> motorIDs;
};

class MotorController {
public:
    std::vector<SerialGroup> serialGroups = {
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9CC6WH-if03-port0", {0,5}},
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9CC6WH-if01-port0", {1,6}},
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9CC6WH-if00-port0", {2, 3, 4}},
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9CC6WH-if02-port0", {7, 8, 9}}
    };
    MotorController() {
        InitializeSerialPorts();
        for(std::array<ThreadData, 4>::iterator td = threadData.begin(); td != threadData.end(); ++td) {
            td->start_time = std::chrono::high_resolution_clock::now();
        }
        // Start the motor control thread
        workerThreads[0] = std::thread(&MotorController::RunThread<0>, this);
        workerThreads[1] = std::thread(&MotorController::RunThread<1>, this);
        workerThreads[2] = std::thread(&MotorController::RunThread<2>, this);
        workerThreads[3] = std::thread(&MotorController::RunThread<3>, this);
        workerThreads[4] = std::thread(&MotorController::MonitorThread, this);
        std::cout << "Start motor thread： Done!" << std::endl;
    }
    ~MotorController() {
        Stop();
        if (dataFile.is_open()) {
            dataFile.close();
        }
    }
    struct ThreadData {
        std::atomic<int> count{0};
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    };

    std::array<ThreadData, 4> threadData;
    std::mutex printMutex;
    std::atomic<bool> running{true};
    std::array<std::mutex, 10> motorMutexes;
    std::mutex fileMutex;
    std::array<std::thread, 5> workerThreads; 

    unitree_hg::msg::dds_::LowCmd_ current_cmd_;
    std::mutex cmd_mutex_;

    void Run(const unitree_hg::msg::dds_::LowCmd_& dds_low_command) {
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            current_cmd_ = dds_low_command;
        }

    }

    void Stop() {
        running = false;
        for(std::thread& thread : workerThreads) {
            if(thread.joinable()) {
                thread.join();
            }
        }
        std::cout << "All worker threads stopped" << std::endl;
    }

public:
    /// Startq（0位偏移）： 左腿roll 内扣，则需增大，右腿内扣则需减小
    std::array<float, 10> Startq ={0.65,  0.45 , 1.28,   0.86,  0.56,
                                   0.8, 0.,  0.301131,  0.513495,  0.2};

    //    std::array<float, 10> Startq ={0.,  0. , 0,   0.0,  0.0, 0.0, -0.0,  0.0,  0.0,  0.0};

    std::array<MotorData, 10> allMotorData;
    float Speed_Ratio = 6.33;
    float Gear_Ratio = 3.;
    std::vector<std::unique_ptr<SerialPort>> serialPorts;

    std::ofstream dataFile;
    std::chrono::time_point<std::chrono::system_clock> lastSaveTime;
    const std::chrono::milliseconds saveInterval{4}; // 100ms保存一次

    void InitializeSerialPorts() {
        for(std::vector<SerialGroup>::iterator group = serialGroups.begin(); group != serialGroups.end(); ++group) {
            std::unique_ptr<SerialPort> port = std::make_unique<SerialPort>(group->port);
            serialPorts.push_back(std::move(port));
        }
    }

    template<int N>
    void RunThread() {

        SerialPort& serial = *serialPorts[N];
        ThreadData& td = threadData[N];
        
         while(running)
           {
            std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
            
            for(std::vector<int>::iterator motorID = serialGroups[N].motorIDs.begin(); 
                motorID != serialGroups[N].motorIDs.end(); ++motorID) {
                MotorCmd cmd;
                MotorData data;
                
                ConfigureMotorCommand(cmd, *motorID, current_cmd_);
                data.motorType = MotorType::GO_M8010_6;
                serial.sendRecv(&cmd, &data);
                ParseMotorFeedback(data, *motorID);
            }
            td.count++;
          }
    }

    void MonitorThread() {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            std::lock_guard<std::mutex> lock(printMutex);
            std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
            
            for(int i = 0; i < 4; ++i) {
                long elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    now - threadData[i].start_time).count();
                int freq = elapsed > 0 ? threadData[i].count / elapsed : 0;
                
                threadData[i].count = 0;
                threadData[i].start_time = now;
            }
    }

    int CalculateChannelID(int motorID) {
        if (motorID == 1) return motorID - 1;
        else if (motorID >= 2 && motorID <= 4) return motorID - 2;
        else if (motorID == 5) return motorID - 4;
        else if (motorID == 6) return motorID - 5;
        else if (motorID >= 7 && motorID <= 9) return motorID - 7;
        return motorID;
    }

    bool IsSpecialMotor(int motorID) const {
        return (motorID == 1 || motorID == 6);
    }

    void PrintFeedback() {
        for (int i = 0; i < 10; ++i) {
            std::cout << "m" << i << ": ";
            if (IsSpecialMotor(i)) {
                std::cout << allMotorData[i].q;
            } else {
                std::cout << allMotorData[i].q;
            }
        }
        std::cout << std::endl;
    }

    void ConfigureMotorCommand(MotorCmd& cmd, int motorID, const unitree_hg::msg::dds_::LowCmd_& dds_low_command) {
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        cmd.id = CalculateChannelID(motorID);
        cmd.kp = dds_low_command.motor_cmd().at(motorID).kp();
        cmd.kd = dds_low_command.motor_cmd().at(motorID).kd();
        cmd.tau = dds_low_command.motor_cmd().at(motorID).tau();
        
        const bool is_special = IsSpecialMotor(motorID);
        const float ratio = is_special ? (Speed_Ratio * Gear_Ratio) : Speed_Ratio;
        
        cmd.q = (dds_low_command.motor_cmd().at(motorID).q() + Startq[motorID]) * ratio;
        cmd.dq = dds_low_command.motor_cmd().at(motorID).dq() * ratio;
    }

    void ParseMotorFeedback(MotorData& data, int motorID) {
        const bool is_special = IsSpecialMotor(motorID);
        const float ratio = is_special ? (Speed_Ratio * Gear_Ratio) : Speed_Ratio;
        
        allMotorData.at(motorID).q = data.q / ratio - Startq[motorID];
        allMotorData.at(motorID).dq = data.dq / ratio;
    }

    const std::array<MotorData, 10> &GetData() const {

        return allMotorData;
    }
};
