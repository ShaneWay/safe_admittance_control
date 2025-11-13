#pragma once
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

class ConfigLoader {
public:
    explicit ConfigLoader(const std::string& filename);
    // 读取基础节点
    YAML::Node getNode(const std::string& key) const;

    // 读取 Vector2d 和 Matrix4d 类型
    Eigen::Vector2d getVector2d(const std::string& key) const;
    Eigen::Matrix4d getMatrix4d(const std::string& key) const;
    Config getConfig() const;

private:
    Config cfg_;
};
