#pragma once
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

class ConfigLoader {
public:
    explicit ConfigLoader(const std::string& filename);
    // 读取基础节点
    YAML::Node getNode(const std::string& key) const;
    YAML::Node getNodeByPath(const std::string& path) const;

    // 读取 Vector2d 和 Matrix4d 类型
    Eigen::Vector2d getVector2d(const YAML::Node node) const;
    Eigen::Matrix2d getMatrix2d(const YAML::Node node) const;
    

private:
    YAML::Node root_;
};
