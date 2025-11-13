#include "ConfigLoader.h"
#include <iostream>

ConfigLoader::ConfigLoader(const std::string& filename) {
   
        YAML::Node root_ = YAML::LoadFile(filename);


}

YAML::Node ConfigLoader::getNode(const std::string& key) const {
    if (!root_[key]) {
        throw std::runtime_error("Key not found in YAML: " + key);
    }
    return root_[key];
}


Eigen::Vector2d ConfigLoader::getVector2d(const std::string& key) const {
    YAML::Node node = getNode(key);
    if (!node.IsSequence() || node.size() != 2) {
        throw std::runtime_error("Invalid Vector2d format for key: " + key);
    }
    Eigen::Vector2d v;
    v << node[0].as<double>(), node[1].as<double>();
    return v;
}

Eigen::Matrix4d ConfigLoader::getMatrix4d(const std::string& key) const {
    YAML::Node node = getNode(key);
    if (!node.IsSequence() || node.size() != 4 || !node[0].IsSequence()) {
        throw std::runtime_error("Invalid Matrix4d format for key: " + key);
    }

    Eigen::Matrix4d m;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            m(i, j) = node[i][j].as<double>();
        }
    }
    return m;
}

Config ConfigLoader::getConfig() const {
    return cfg_;
}
