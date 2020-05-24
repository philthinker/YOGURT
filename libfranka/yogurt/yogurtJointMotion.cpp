// yogurtJointMotion
// These codes are used to practice joint motion generation of Franka Panda.
// 2020.05.23

#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>


int main(int argc, char** argv) {
    if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " 172.16.0.2" << std::endl;
    return -1;
  }
  try
  {
      franka::Robot robot(argv[1]);
      
      robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
      robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
      robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

      std::cout << "WARNING: This code will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
      std::cin.ignore();

      // Set additional parameters always before the control loop, NEVER in the control loop!
      // Set collision behavior.
      /*
      robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
      */
      
      std::array<double, 7> initial_position;
      double time = 0.0;
      robot.control([&initial_position, &time](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }
      double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));

      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2], initial_position[3],
                                        initial_position[4], initial_position[5],
                                        initial_position[6] + delta_angle}};
      //std::cout << time << std::endl;
      if (time >= 5) {
        std::cout << std::endl << "Finished motion, shutting down yogurt" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
    
  }
  catch(const franka::Exception& e)
  {
      std::cerr << e.what() << std::endl;
      return -1;
  }
  return 0;
}