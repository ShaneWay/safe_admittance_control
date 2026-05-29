#pragma once

#include "BaseController.h"
#include "DataLogger.h"

#include "NormalController.h"
#include "SmcController.h"
#include "SfcController.h"
#include "KikuuweController.h"
#include "KikModify.h"
#include "DismcController.h"

#include <memory>
#include <stdexcept>
#include <string>

class ControllerFactory
{
  public:
    ControllerFactory(const std::string& config_file, DataLogger& logger) : config_file_(config_file), logger_(logger) {}

    std::unique_ptr<BaseController> create(const std::string& type)
    {
        if (type == "normal") {
            return std::make_unique<NormalController>(config_file_, logger_);
        }

        if (type == "smc") {
            return std::make_unique<SmcController>(config_file_, logger_);
        }

        if (type == "sfc") {
            return std::make_unique<SfcController>(config_file_, logger_);
        }

        if (type == "kik") {
            return std::make_unique<KikuuweController>(config_file_, logger_);
        }

        if (type == "kikc") {
            return std::make_unique<KikModify>(config_file_, logger_);
        }

        if (type == "dismc") {
            return std::make_unique<DismcController>(config_file_, logger_);
        }

        throw std::runtime_error("Unknown controller type: " + type);
    }

  private:
    std::string config_file_;
    DataLogger& logger_;
};