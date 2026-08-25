// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace behavior {

/// Expected input and output tensor shapes for a policy model.
struct OnnxModelContract {
    std::vector<int64_t> expected_input_shape;
    std::vector<int64_t> expected_output_shape;
};

/// Tensor shapes reported by the loaded ONNX model.
struct OnnxModelInfo {
    std::vector<int64_t> input_shape;
    std::vector<int64_t> output_shape;
};

/// Small validated wrapper around an ONNX Runtime inference session.
class OnnxSession {
public:
    OnnxSession(
        const std::filesystem::path& model_path,
        const OnnxModelContract& contract);

    const OnnxModelInfo& info() const;
    /// Runs one flattened float input and returns the flattened model output.
    std::vector<float> run(const std::vector<float>& input) const;

    static YAML::Node load_yaml(const std::filesystem::path& yaml_path);

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
    OnnxModelInfo info_;
};

}  // namespace behavior
