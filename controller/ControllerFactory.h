#pragma once
#include "BaseController.h"
#include "SmcController.h"
#include "NormalController.h"
#include "SfcController.h"
#include "KikuuweController.h"
#include <memory>

class ControllerFactory {
public:
    static std::unique_ptr<BaseController> create(const ControllerConfig& cfg) {
        if (cfg.control_mode == "normal") {
            return std::make_unique<NormalController>(cfg);
        } else if (cfg.control_mode == "smc") {
            return std::make_unique<SmcController>(cfg);
        } else if (cfg.control_mode == "sfc") {
            return std::make_unique<SfcController>(cfg);
        } else if (cfg.control_mode == "kikuuwe") {
            return std::make_unique<KikuuweController>(cfg);
        }
    }
};