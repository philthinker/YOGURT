//yogurtReadOnce
// These codes are used to read the RobotState structure once only
// 2020.06.09

#include <iostream>
#include <cmath>
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
    }
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
        // Read the current state
        franka::RobotState state = robot.readOnce();
        // export a csv file
        std::ofstream outFile;
        outFile.open("robot_state.csv", std::ios::out);
        // 1st row: Joint pose
        outFile << state.q[0] << ',' 
                << state.q[1] << ',' 
                << state.q[2] << ',' 
                << state.q[3] << ','
                << state.q[4] << ','
                << state.q[5] << ','
                << state.q[6] << std::endl;
        // 2nd row: Cartesian pose
        
        outFile.close();
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    return 0;
}