#include "ConfigLoader.h"
#include <iostream>

ConfigLoader::ConfigLoader(const std::string& filename) {
   
         root_ = YAML::LoadFile(filename);
}

YAML::Node ConfigLoader::getNode(const std::string& key) const {
    if (!root_[key]) {
        throw std::runtime_error("Key not found in YAML: " + key);
    }
    return root_[key];
}

YAML::Node ConfigLoader::getNodeByPath(const std::string& path) const {
    YAML::Node node = root_;
    std::cout << "root1:" << root_ << std::endl;
    std::stringstream ss(path);
    std::string key;
    int count = 0;
    while (std::getline(ss, key, '.')) {
        std::cout << "key:" << key << std::endl;
        std::cout << "[getNodeByPath] step " << count++ << " -> " << key << "\n";
        node = node[key];
        std::cout << node << std::endl;
        if (!node)
            throw std::runtime_error("YAML path not found: " + path);
    }
    std::cout << "root2:" << root_ << std::endl;
    return node;
}

Eigen::Vector2d ConfigLoader::getVector2d(const YAML::Node node) const {
    // YAML::Node node = getNodeByPath(key);
    Eigen::Vector2d v;
    v << node[0].as<double>(), node[1].as<double>();
    return v;
}

Eigen::Matrix2d ConfigLoader::getMatrix2d(const YAML::Node node) const {
    // YAML::Node node = getNodeByPath(key);
    // if (!node.IsSequence() || node.size() != 4 ) {
    //     throw std::runtime_error("Invalid Matrix2d format for key: " + key);
    // }

    Eigen::Matrix2d M;
    // for (size_t i = 0; i < 2; ++i) {
    //     for (size_t j = 0; j < 2; ++j) {
    //         m(i, j) = node[i][j].as<double>();
    //     }
    // }
    if (node.IsSequence() && node.size() == 2 &&
        node[0].IsSequence() && node[0].size() == 2 &&
        node[1].IsSequence() && node[1].size() == 2) {

        M(0,0) = node[0][0].as<double>();
        M(0,1) = node[0][1].as<double>();
        M(1,0) = node[1][0].as<double>();
        M(1,1) = node[1][1].as<double>();
        return M;
    }
    return M;
}

