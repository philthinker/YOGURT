// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/examples.h>
#include <ctime>
#include <fstream>
#include <sstream>

#define Time_Max 180
using namespace std;

int main(int argc, char** argv) {
 try {
    franka::Robot robot("172.16.0.2");
        setDefaultBehavior(robot);
    // First move the robot to a suitable joint configuration
     double data[700][8]={0};

     ifstream fp;
     fp.open("data_point.txt");
     string sline;
     double s1,s2,s3,s4,s5,s6,s7;
     int num=0;
     while(getline(fp,sline))
     {
     	istringstream sin(sline);
     	sin>>s1>>s2>>s3>>s4>>s5>>s6>>s7;
     	data[num][0]=s1;data[num][1]=s2;data[num][2]=s3;data[num][3]=s4;data[num][4]=s5;data[num][5]=s6;data[num][6]=s7;
     	num++;
     	// cout <<s1<<s2<<s3<<s4<<s5<<s6<<s7<<endl; 
     }
     // for(int i=0;i<100;i++)
     // {
     // 	for(int j=0;j<8;j++)
     // 		cout << data[i][j] <<" ";
     // 	cout << endl;
     // }



 try {

    array<double, 7> q_goal = {{data[0][0],data[0][1],data[0][2],data[0][3],data[0][4],data[0][5],data[0][6]}};
    MotionGenerator motion_generator(0.1, q_goal);
    cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << endl
              << "Press Enter to continue..." << endl;
    cin.ignore();
    robot.control(motion_generator);
    cout << "Finished moving to initial joint configuration." << endl;
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
            for(int i=1;i<50000;i++)
            for(int j=1;j<20000;j++);
    }catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running1 error recovery..." << std::endl;
        robot.automaticErrorRecovery();
        return 0;
      }

    // robot.setCollisionBehavior(
    //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
    //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{60.0,60.0, 54.0, 54.0, 48.0, 42.0, 36.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{60.0,60.0, 54.0, 54.0, 48.0, 42.0, 36.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{60.0, 60.0, 60.0, 75.0, 75.0, 75.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{60.0, 60.0, 60.0, 75.0, 75.0, 75.0}});

            // for(int i=1;i<50000;i++)
            // for(int j=1;j<50000;j++);
    array<double, 7> initial_position;
    double time = 0.0;
     	
 try {

    robot.control([&initial_position, &time, &data](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
	time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }
      double delta_angle0 = 0.0,delta_angle1 = 0.0,delta_angle2 = 0.0,delta_angle3 = 0.0,delta_angle4 = 0.0,delta_angle5 = 0.0,delta_angle6 = 0.0;
      
      int x= time*1000;

        delta_angle0 = (data[x/1000+1][0]-data[x/1000][0])/2 * (1 - cos(M_PI * (time-x/1000))) + (data[x/1000][0]-data[0][0]);
        delta_angle1 = (data[x/1000+1][1]-data[x/1000][1])/2 * (1 - cos(M_PI * (time-x/1000))) + (data[x/1000][1]-data[0][1]);
        delta_angle2 = (data[x/1000+1][2]-data[x/1000][2])/2 * (1 - cos(M_PI * (time-x/1000))) + (data[x/1000][2]-data[0][2]);
        delta_angle3 = (data[x/1000+1][3]-data[x/1000][3])/2 * (1 - cos(M_PI * (time-x/1000))) + (data[x/1000][3]-data[0][3]);
        delta_angle4 = (data[x/1000+1][4]-data[x/1000][4])/2 * (1 - cos(M_PI * (time-x/1000))) + (data[x/1000][4]-data[0][4]);
        delta_angle5 = (data[x/1000+1][5]-data[x/1000][5])/2 * (1 - cos(M_PI * (time-x/1000))) + (data[x/1000][5]-data[0][5]);
        delta_angle6 = (data[x/1000+1][6]-data[x/1000][6])/2 * (1 - cos(M_PI * (time-x/1000))) + (data[x/1000][6]-data[0][6]);

      cout<< x/1000 << " " << time-x/1000 << endl;
      franka::JointPositions output = {{initial_position[0]+ delta_angle0, initial_position[1]+ delta_angle1,
                                        initial_position[2]+ delta_angle2, initial_position[3]+ delta_angle3,
                                        initial_position[4]+ delta_angle4, initial_position[5]+ delta_angle5,
                                        initial_position[6]+ delta_angle6}};
      if (time >= Time_Max ) {
        cout << endl << "Finished motion, shutting down example" << endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
    
    }catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running2 error recovery..." << std::endl;
        robot.automaticErrorRecovery();
      }

  } catch (const franka::Exception& e) {
    cout << e.what() << endl;
    return -1;
  }
  return 0;
}