// yogurtTemplate
// 
// Haopeng Hu
// 2020.05.29
// All rights reserved

// System 
#include<iostream>
#include<cmath>

// franka
#include<franka/robot.h>
#include<franka/exception.h>
#include<franka/model.h>

int main(int argc, char** argv){
    // Check arguments
    // The fci-ip address always comes frist.
    if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " 172.16.0.2" << std::endl;
    return -1;
    try
    {
        franka::Robot robot(argv[1]);
        // Set default param.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
        // Wait for the keyboard command to run the program
        std::cout << "Make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue. Good luck!" << std::endl;
        std::cin.ignore();
        // Control strategy
        // Policy parameters
        double time = 0.0;
        std::array<double, 7> initialPosition;
        // Control loop
        /*
        robot.control([&initialPosition, &time](const franka::RobotState& robotState,
                                franka::Duration& period) -> franka::JointPositions {   // Control mode
            time += period.toSec();
            // Control policy
            franka::JointPositions output = {{initialPosition[0], initialPosition[1],
                                        initialPosition[2], initialPosition[3],
                                        initialPosition[4], initialPosition[5],
                                        initialPosition[6]}};
            // Terminal condition
            if (time > 10)
            {
                std::cout << "Finish the motion. Shut down!" << std::endl;
                return franka::MotionFinished(output);
            }
            return output;
        });
        */
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    return 0;
}