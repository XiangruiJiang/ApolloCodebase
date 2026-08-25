// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 The Hong Kong University of Science and Technology (Guangzhou), Humanoid Computing & Learning Lab.

#pragma once

#include "src/decision/high_level_command.h"

#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace decision::bt {

/// Result of ticking a behavior-tree node, optionally carrying a command.
template <typename Context>
struct NodeResult {
    bool success{false};
    std::optional<HighLevelCommand> command;

    static NodeResult failure() {
        return {};
    }

    static NodeResult success_only() {
        NodeResult result;
        result.success = true;
        return result;
    }

    static NodeResult with_command(HighLevelCommand command) {
        NodeResult result;
        result.success = true;
        result.command = std::move(command);
        return result;
    }
};

/// Polymorphic interface implemented by all behavior-tree nodes.
template <typename Context>
class Node {
public:
    virtual ~Node() = default;
    virtual NodeResult<Context> tick(Context& context) const = 0;
};

template <typename Context>
using NodePtr = std::shared_ptr<const Node<Context>>;

/// Leaf node backed by a callable.
template <typename Context>
class LambdaNode final : public Node<Context> {
public:
    explicit LambdaNode(std::function<NodeResult<Context>(Context&)> tick)
        : tick_(std::move(tick)) {}

    NodeResult<Context> tick(Context& context) const override {
        return tick_(context);
    }

private:
    std::function<NodeResult<Context>(Context&)> tick_;
};

/// Runs children in order until one fails or emits a command.
template <typename Context>
class SequenceNode final : public Node<Context> {
public:
    explicit SequenceNode(std::vector<NodePtr<Context>> children)
        : children_(std::move(children)) {}

    NodeResult<Context> tick(Context& context) const override {
        for (const auto& child : children_) {
            const auto result = child->tick(context);
            if (!result.success) {
                return NodeResult<Context>::failure();
            }
            if (result.command.has_value()) {
                return result;
            }
        }
        return NodeResult<Context>::success_only();
    }

private:
    std::vector<NodePtr<Context>> children_;
};

/// Runs children in order until one succeeds.
template <typename Context>
class FallbackNode final : public Node<Context> {
public:
    explicit FallbackNode(std::vector<NodePtr<Context>> children)
        : children_(std::move(children)) {}

    NodeResult<Context> tick(Context& context) const override {
        for (const auto& child : children_) {
            const auto result = child->tick(context);
            if (result.success) {
                return result;
            }
        }
        return NodeResult<Context>::failure();
    }

private:
    std::vector<NodePtr<Context>> children_;
};

/// Creates a sequence composite from `children`.
template <typename Context>
NodePtr<Context> sequence(std::vector<NodePtr<Context>> children) {
    return std::make_shared<SequenceNode<Context>>(std::move(children));
}

/// Creates a fallback composite from `children`.
template <typename Context>
NodePtr<Context> fallback(std::vector<NodePtr<Context>> children) {
    return std::make_shared<FallbackNode<Context>>(std::move(children));
}

/// Creates a condition node that succeeds when `predicate` is true.
template <typename Context>
NodePtr<Context> condition(std::function<bool(const Context&)> predicate) {
    return std::make_shared<LambdaNode<Context>>(
        [predicate = std::move(predicate)](Context& context) {
            return predicate(context) ? NodeResult<Context>::success_only() : NodeResult<Context>::failure();
        });
}

/// Creates an action node that always emits the callable's command.
template <typename Context>
NodePtr<Context> command(std::function<HighLevelCommand(Context&)> action) {
    return std::make_shared<LambdaNode<Context>>(
        [action = std::move(action)](Context& context) {
            return NodeResult<Context>::with_command(action(context));
        });
}

/// Creates a leaf node from a callable returning a complete node result.
template <typename Context>
NodePtr<Context> task(std::function<NodeResult<Context>(Context&)> action) {
    return std::make_shared<LambdaNode<Context>>(std::move(action));
}

}  // namespace decision::bt
