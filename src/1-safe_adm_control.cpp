#include <iostream>

#include "KortexRobotInterface.h"
#include "SafeAdmittanceExperiment.h"

int main(int argc, char** argv)
{
    KortexRobotInterface robot(argc, argv);

    if (!robot.connect()) {
        std::cout << "Failed to connect Kortex robot." << std::endl;
        return -1;
    }

    if (!robot.moveToExperimentStart()) {
        std::cout << "Failed to move to experiment start position." << std::endl;
        robot.disconnect();
        return -1;
    }

    SafeAdmittanceExperiment experiment(robot.base(), robot.baseCyclic(), robot.actuatorConfig());

    const bool success = experiment.run();

    robot.disconnect();

    if (!success) {
        std::cout << "There has been an unexpected error." << std::endl;
        return -1;
    }

    return 0;
}