//kambuchaReadOnce
//  Read the RobotState once
//  You can specify a tail name to identify the robot state file
//
//  Haopeng Hu
//  2020.06.28
//
//  What do you want?


#include <iostream>
#include <fstream>
#include <string>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/model.h>

int main(int argc, char** argv){
    if(argc < 2){
        // No fci-ip given
        std::cerr << "Usage: " << argv[0] << " <fci-ip>" << std::endl;
        return -1;
    }
        std::cout << "Read the robot state once" << std::endl << "Press Enter to continue" << std::endl;
        std::cin.ignore();
        // Init. file
        std::string tailName("tmp.csv");
        if(argc >= 3){
            tailName.assign(argv[2]);
            tailName.append(".csv");
        }
        std::string fileName("RobotState_");
        fileName.insert(11,tailName);
        std::ofstream fileOut;
        fileOut.open(fileName,std::ios::out);
    try{
        // Init. Robot
        franka::Robot robot(argv[1]);
        franka::RobotState state = robot.readOnce();
        franka::Model model(robot.loadModel());
        // What do you want?
        // Joint Position
        fileOut << "JointPosition";
        for (int i = 0; i < 6; i++)
        {
            fileOut << ',' << state.q[i];
        }
        fileOut << std::endl;

        /* Cartesian pose
        O: base frame
        F: flange frame
        NE: nomimal end effector frame
        EE: end effector frame
        K: stiffness frame
        */  
        // End effector pose in base frame
        fileOut << "OTEE";
        for (int i = 0; i < 15; i++)
        {
            fileOut << ',' << state.O_T_EE[i];
        }
        fileOut << std::endl;

        // End effector frame pose in flange frame
        fileOut << "FTEE";
        for (int i = 0; i < 15; i++)
        {
            fileOut << ',' << state.F_T_EE[i];
        }
        fileOut << std::endl;

        // Nominal end effector frame pose in flange frame
        fileOut << "FTNE";
        for (int i = 0; i < 15; i++)
        {
            fileOut << ',' << state.F_T_NE[i];
        }
        fileOut << std::endl;

        // Nominal end effector frame pose in flange frame
        fileOut << "NETEE";
        for (int i = 0; i < 15; i++)
        {
            fileOut << ',' << state.NE_T_EE[i];
        }
        fileOut << std::endl;

        // Stiffness frame pose in end effector frame
        fileOut << "EETK";
        for (int i = 0; i < 15; i++)
        {
            fileOut << ',' << state.EE_T_K[i];
        }
        fileOut << std::endl;

        // Force
        // Estimated wrench on stiffness frame w.r.t. base frame
        fileOut << "OFK";
        for (int i = 0; i < 5; i++)
        {
            fileOut << ',' << state.O_F_ext_hat_K[i];
        }
        fileOut << std::endl;
        // Estimated wrench on stiffness frame w.r.t. stiffness frame
        fileOut << "KFK";
        for (int i = 0; i < 5; i++)
        {
            fileOut << ',' << state.K_F_ext_hat_K[i];
        }
        fileOut << std::endl;

        // Torque
        fileOut << "JointTorque";
        for (int i = 0; i < 7; i++)
        {
            fileOut << ',' << state.tau_J[i];
        }
        fileOut << std::endl;

        fileOut.close();
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        fileOut.close();
        return -1;
    }
    return 0;
}