//
// Created by cyy on 2020/12/27.
//

#include <iostream>
#include <vector>
#include <chrono>
#include <string>
#include "onnx/onnxruntime_cxx_api.h"
#include <ctime>
#include <sys/time.h>
#include <eigen3/Eigen/Dense>
#include <assert.h>
using namespace Eigen;
using namespace std;

class OnnxInference {
public:
    explicit OnnxInference() = default;

public:
    int64_t input_dim = 0;
    int64_t output_dim = 0;
    int64_t stack_dim = 0;

private:
    std::string _modelPath;
    std::vector<const char *> input_node_names = {"input"};
    std::vector<const char *> output_node_names = {"output"};
    std::vector<int64_t> input_node_dims;
    size_t input_tensor_size = 0;

public:
    void init(int obs_space, int act_space, int stack_space) {
        input_dim = obs_space;
        output_dim = act_space;
        stack_dim = stack_space;
        input_node_dims = {1, input_dim * stack_dim};
        input_tensor_size = input_node_dims.at(0) * input_node_dims.at(1);
        //        cout << "input_tensor_size: " << input_tensor_size << endl;
    }

    Matrix<float, Dynamic, 1> inference(Ort::Session *session, Matrix<float, Dynamic, 1> observation) {
        std::vector<float> input_tensor_values(input_tensor_size);
        for (unsigned int i = 0; i < input_tensor_size; i++)
            input_tensor_values[i] = observation[i];
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(),
                                                                  input_tensor_size, input_node_dims.data(), 2);
        std::vector<Ort::Value> ort_inputs;
        ort_inputs.push_back(std::move(input_tensor));
        auto output_tensors = session->Run(Ort::RunOptions{nullptr}, input_node_names.data(), ort_inputs.data(),
                                           ort_inputs.size(), output_node_names.data(), 1);
        float *outputData = output_tensors[0].GetTensorMutableData<float>();
        Eigen::Map<Eigen::Matrix<float, -1, 1> > net_out(outputData, output_dim, 1);
        auto net_out_action = net_out.cwiseMax(-1).cwiseMin(1.);
        // cout << "net_true_action: " << net_out_action.transpose() << endl;
        return net_out_action;
    }
};
