// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#include "src/behavior/onnx_session.h"

#include <onnxruntime_cxx_api.h>

#include <stdexcept>

namespace behavior {

struct OnnxSession::Impl {
    explicit Impl(const std::filesystem::path& model_path)
        : session(env(), model_path.c_str(), make_options()) {
        Ort::AllocatorWithDefaultOptions allocator;
        input_name = session.GetInputNameAllocated(0U, allocator).get();
        output_name = session.GetOutputNameAllocated(0U, allocator).get();
    }

    static Ort::Env& env() {
        static Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "ApolloCodeBase"};
        return env;
    }

    static Ort::SessionOptions make_options() {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
        return options;
    }

    Ort::Session session;
    std::string input_name;
    std::string output_name;
};

namespace {

std::vector<int64_t> tensor_shape(Ort::Session& session, std::size_t index, bool input) {
    Ort::TypeInfo type_info =
        input ? session.GetInputTypeInfo(index) : session.GetOutputTypeInfo(index);
    return type_info.GetTensorTypeAndShapeInfo().GetShape();
}

std::size_t tensor_size(const std::vector<int64_t>& shape) {
    std::size_t size = 1U;
    for (const int64_t dim : shape) {
        if (dim <= 0) {
            throw std::runtime_error("ONNX tensor shape must be fully static");
        }
        size *= static_cast<std::size_t>(dim);
    }
    return size;
}

void verify_shape(
    const std::vector<int64_t>& actual,
    const std::vector<int64_t>& expected,
    const char* label) {
    if (actual != expected) {
        throw std::runtime_error(
            std::string("ONNX ") + label + " shape mismatch");
    }
}

}  // namespace

OnnxSession::OnnxSession(
    const std::filesystem::path& model_path,
    const OnnxModelContract& contract)
    : impl_(nullptr) {
    if (!std::filesystem::exists(model_path)) {
        throw std::runtime_error("ONNX model not found: " + model_path.string());
    }
    impl_ = std::make_shared<Impl>(model_path);
    info_.input_shape = tensor_shape(impl_->session, 0U, true);
    info_.output_shape = tensor_shape(impl_->session, 0U, false);

    verify_shape(info_.input_shape, contract.expected_input_shape, "input");
    verify_shape(info_.output_shape, contract.expected_output_shape, "output");
}

const OnnxModelInfo& OnnxSession::info() const {
    return info_;
}

std::vector<float> OnnxSession::run(const std::vector<float>& input) const {
    if (input.empty()) {
        throw std::runtime_error("ONNX input must not be empty");
    }
    const std::size_t expected_size = tensor_size(info_.input_shape);
    if (input.size() != expected_size) {
        throw std::runtime_error("ONNX input size mismatch");
    }

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<int64_t> input_shape = info_.input_shape;
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        const_cast<float*>(input.data()),
        input.size(),
        input_shape.data(),
        input_shape.size());

    const char* input_names[] = {impl_->input_name.c_str()};
    const char* output_names[] = {impl_->output_name.c_str()};
    auto outputs = impl_->session.Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1U,
        output_names,
        1U);

    auto& output_tensor = outputs.front();
    const float* output_data = output_tensor.GetTensorData<float>();
    const auto output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
    std::size_t output_size = 1U;
    for (int64_t dim : output_shape) {
        output_size *= static_cast<std::size_t>(dim);
    }
    return std::vector<float>(output_data, output_data + output_size);
}

YAML::Node OnnxSession::load_yaml(const std::filesystem::path& yaml_path) {
    if (!std::filesystem::exists(yaml_path)) {
        throw std::runtime_error("YAML asset not found: " + yaml_path.string());
    }
    return YAML::LoadFile(yaml_path.string());
}

}  // namespace behavior
