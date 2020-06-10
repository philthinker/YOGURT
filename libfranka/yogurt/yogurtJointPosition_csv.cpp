//yogurtJointPosition_csv
// Haopeng Hu
// 2020.06.10

#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

int main(int argc, char** argv) {
    if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " 172.16.0.2" << std::endl;
    return -1;
  }
  try
  {
      // Read the csv file
      std::ifstream trajJP("dataJP2_empirical.csv",std::ios::in);
      if(trajJP.fail()){
            std::cout << "File not found" << std::endl;
            return -1;
        }
        std::string line;
        double s1, s2, s3, s4, s5, s6, s7;
        char c1, c2, c3, c4, c5, c6;
        size_t N = 6145;
        double dataJP[N][7]; // 5164 rows and 7 columns
        int num = 0;
        while(std::getline(trajJP,line) && trajJP.good()){
            std::istringstream sin(line);
            sin >> s1 >> c1 >> s2 >> c2 >> s3 >> c3 >> s4 >> c4 >> s5 >> c5 >> s6 >> c6 >> s7;
            dataJP[num][0]=s1;
            dataJP[num][1]=s2;
            dataJP[num][2]=s3;
            dataJP[num][3]=s4;
            dataJP[num][4]=s5;
            dataJP[num][5]=s6;
            dataJP[num][6]=s7;
            num++;
        }
      // Init. the robot
      franka::Robot robot(argv[1]);
      // Set additional parameters always before the control loop, NEVER in the control loop!
      // Set collision behavior.
      robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
      robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
      robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

      std::cout << "WARNING: This code will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue. Good luck!" << std::endl;
      std::cin.ignore();

      // Move to initial position of the csv file
      std::array<double, 7> q_goal = {{dataJP[0][0], dataJP[0][1], dataJP[0][2], dataJP[0][3], dataJP[0][4], dataJP[0][5], dataJP[0][6]}};
      MotionGenerator motion_generator(0.5, q_goal);
      robot.control(motion_generator);
      std::cout << "Come to the initial configuration" << std::endl;

      std::array<double, 7> initial_position;
      size_t index = 0;
      robot.control([&initial_position, &index, &dataJP, N](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      franka::JointPositions output = {{dataJP[index][0], 
                                        dataJP[index][1],
                                        dataJP[index][2], 
                                        dataJP[index][3],
                                        dataJP[index][4], 
                                        dataJP[index][5],
                                        dataJP[index][6]}};
      if (index >= (N-2)) { // I DO NOT know why currently ...
        return franka::MotionFinished(output);
      }
      index ++;
      return output;
    });
    std::cout << std::endl << "Finished motion, shutting down yogurt" << std::endl;
  }
  catch(const franka::Exception& e)
  {
      std::cerr << e.what() << std::endl;
      return -1;
  }
  return 0;
}