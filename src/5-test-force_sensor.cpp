#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

#include "sriCommDefine.h"
#include "sriCommManager.h"

int main()
{
    std::cout << "==========================================" << std::endl;
    std::cout << "Force sensor thread close test" << std::endl;
    std::cout << "==========================================" << std::endl;

    CSRICommManager commManager;

    std::cout << "[1] Init force sensor..." << std::endl;

    if (!commManager.Init()) {
        std::cout << "[ERROR] Init failed." << std::endl;
        return -1;
    }

    std::cout << "[OK] Init success." << std::endl;

    std::cout << "[2] Run force sensor communication..." << std::endl;

    if (!commManager.Run()) {
        std::cout << "[ERROR] Run failed." << std::endl;
        return -1;
    }

    std::cout << "[OK] Run success. Receiver thread should be running." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "[3] Send GSD command..." << std::endl;

    if (!commManager.SendGODCommand("GSD", "")) {
        std::cout << "[WARNING] SendGODCommand(\"GSD\", \"\") returned false." << std::endl;
    } else {
        std::cout << "[OK] SendGODCommand success." << std::endl;
    }

    std::cout << "[4] Read raw force data for 5 seconds..." << std::endl;
    std::cout << "Please press the force sensor by hand and observe whether values change." << std::endl;

    for (int i = 0; i < 50; ++i) {
        std::cout << "force[" << i << "] = " << commManager.force[0] << " " << commManager.force[1] << " " << commManager.force[2] << " "
                  << commManager.force[3] << " " << commManager.force[4] << " " << commManager.force[5] << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[5] Stop force sensor communication..." << std::endl;

    if (!commManager.Stop()) {
        std::cout << "[ERROR] Stop failed." << std::endl;
        return -1;
    }

    std::cout << "[OK] Stop success." << std::endl;

    // 等一会儿，验证后台线程是否真的已经退出
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "==========================================" << std::endl;
    std::cout << "Force sensor thread close test finished." << std::endl;
    std::cout << "If no terminate/abort appears after this line, thread close is OK." << std::endl;
    std::cout << "==========================================" << std::endl;

    return 0;
}