#pragma once
#include "BaseController.h"
#include <ControlState.h>
#include <BaseParams.h>
#include "SmcController.h"
#include "NormalController.h"
#include "SfcController.h"
#include "KikuuweController.h"
#include "DismcController.h"
#include <memory>

class ControllerFactory {
public:
    ControllerFactory(BaseParams& params, ControlState& state)
        : params_(params), state_(state) {}
    std::unique_ptr<BaseController> create(const std::string& type) {
        if (type == "normal") {
            return std::make_unique<NormalController>(params_, state_);
        } else if (type == "smc") {
            return std::make_unique<SmcController>(params_, state_);
        } else if (type == "sfc") {
            return std::make_unique<SfcController>(params_, state_);
        } else if (type == "kik") {
            return std::make_unique<KikuuweController>(params_, state_);
        } else if (type == "dismc") {
            return std::make_unique<DismcController>(params_, state_);
        }
        
        
    }
private:
    BaseParams& params_;
    ControlState& state_;
};