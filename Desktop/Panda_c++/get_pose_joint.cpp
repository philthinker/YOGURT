// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <fstream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <cmath>
#include <ctime>


using namespace std;
int main(int argc, char** argv) 
{
    franka::Robot robot("172.16.0.2");
    std::array<double, 16> pose;
    std::array<double, 7> joint;
    time_t timep;
	char tmp[64];
	while(1)
	{
		time(&timep);
		strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
		franka::RobotState state = robot.readOnce();
		pose = state.O_T_EE;
		joint= state.q;
		ofstream pose_data("pose.txt",ios::app);
		ofstream joint_data("joint.txt",ios::app);

		// pose_data << tmp << " ";
		for(auto i : pose){cout << i << " ";pose_data << i << " ";}
		cout  << endl;pose_data << endl;
		// joint_data << tmp << " ";
		for(auto i : joint){cout << i << " ";joint_data << i << " ";}
		cout  << endl;joint_data << endl;
		for(int i=1;i<10000;i++)
			for(int j=1;j<1000;j++);
		pose_data.close();
		joint_data.close();
	}
}

