//yogurtGetJP_rt
// Record the 7th dim. of joint position of franka to csv file
//
// Haopeng Hu
// 2020.06.07, HIT's 100th anniversary
// All rights reserved

#include <iostream>
#include <cmath>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>

#include <franka/robot.h>
#include <franka/exception.h>

int main(int argc, char** argv){
    // Check arguments
    // The fci-ip address always comes frist.
    if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " 172.16.0.2" << std::endl;
    return -1;
    }
    // Init. file
    std::ofstream out_file;
    out_file.open("dataJP.csv",std::ios::out);
    try
    {
        franka::Robot robot(argv[1]);
        // Set default param.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        // Note that you cannot set impedance in the teaching mode
        //robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        //robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
        // Wait for the keyboard command to run the program
        std::cout << "Make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue. Good luck!" << std::endl;
        std::cin.ignore();
        // Read strategy (1kHz)
        size_t count = 0;
        unsigned int subcount = 0;
        robot.read([&count,&subcount,&out_file](const franka::RobotState& robot_state) -> bool {
            subcount++;
            // Set the threshold to lower the fps
            if(subcount >= 100){ // 10 per second
                subcount = 0;
                count++;
                out_file << robot_state.q[0] << ','
                    << robot_state.q[1] << ','
                    << robot_state.q[2] << ','
                    << robot_state.q[3] << ','
                    << robot_state.q[4] << ','
                    << robot_state.q[5] << ','
                    << robot_state.q[6] << std::endl;
            }
            return count < 200; // timeout
        });
        out_file.close();
        return 0;
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        out_file.close();
        return -1;
    }
}