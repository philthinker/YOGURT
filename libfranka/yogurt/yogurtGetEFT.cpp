//yogurtGetEFT
// Record the external 6th dim. of force and torque by franka's internal dynamics
//
// Haopeng Hu
// 2020.06.07, HIT's 100th anniversary
// All rights reserved

#include <iostream>
#include <cmath>
#include <ctime>
#include <fstream>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/model.h>

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
        // Read strategy
        
        return 0;
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    
}