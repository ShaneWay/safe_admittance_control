#pragma once
#include "BaseController.h"
#include "SmcController.h"
#include "NormalController.h"
#include "SfcController.h"
#include "KikuuweController.h"
#include <memory>

class ControllerFactory {
public:
    static std::unique_ptr<BaseController> create(const std::string& type, const ConfigLoader& loader) {
        if (type == "normal") {
            return std::make_unique<NormalController>(loader);
        } else if (type == "smc") {
            return std::make_unique<SmcController>(loader);
        } else if (type == "sfc") {
            return std::make_unique<SfcController>(loader);
        } else if (type == "kik") {
            return std::make_unique<KikuuweController>(loader);
        }
    }
};