//kambuchaJointPoseRT
//  Move to given joint pose
//  You can specify a file name for the given joint poses
//
//  Haopeng Hu
//  2020.06.30
//  
//  kambuchaJointPose <fci-ip> fileInName speed fileOutName fps

#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/model.h>

#include "examples_common.h"

int main(int argc, char** argv){
    if(argc < 3){
        std::cerr << "Usage: " << argv[0] << " <fci-ip> " << "fileInName " << "speed " << "fileOutName " << " fps " << std::endl;
        return -1;
    }
    // Speed
    double speed = 0.5; // (0,1], default: 0.5
    if(argc >= 4){
        speed = std::stod(argv[3]);
        if(speed < 0.1){
            speed = 0.1;
        }else if(speed > 1){
            speed = 1;
        }
    }
    // Output file name
    std::string fileOutName("RobotState");
    if(argc >= 5){
        fileOutName.assign(argv[4]);
    }
        // OTEE
        std::string fileOTEEName(fileOutName);  // Deep copy
        std::ofstream fileOTEE(fileOTEEName.append("_OTEE.csv"),std::ios::out);
        // tauJ
        std::string fileTauJName(fileOutName);
        std::ofstream fileTauJ(fileTauJName.append("_tauJ.csv"),std::ios::out);
        // q
        std::string fileJPName(fileOutName);
        std::ofstream fileJP(fileJPName.append("_JP.csv"),std::ios::out);
        // OFK
        std::string fileOFKName(fileOutName);
        std::ofstream fileOFK(fileOFKName.append("_OFK.csv"),std::ios::out);
        // KFK
        std::string fileKFKName(fileOutName);
        std::ofstream fileKFK(fileKFKName.append("_KFK.csv"),std::ios::out);
    // Read the goal joint poses into vectors
    std::string fileInName(argv[2]);
    fileInName.append(".csv");            // fileInName.csv
    std::ifstream fileIn(fileInName,std::ios::in);
    if(fileIn.fail()){
        std::cerr << "File: " << argv[3] << ".csv not found" << std::endl;
        return -1;
    }
    std::vector<std::vector<double>> jointPoseData; // Store the Cartesian pose data
    std::string line;                               // Store one line of data for file reading
    while (std::getline(fileIn,line) && fileIn.good())
    {
        std::istringstream dataIn(line);
        std::string dataTmp;
        std::vector<double> jointPoseDataTmp;       // Store one Cartesian pose data
        while (std::getline(dataIn,dataTmp,','))
        {
            jointPoseDataTmp.push_back(std::stod(dataTmp));
        }
        jointPoseData.push_back(jointPoseDataTmp);
    }
    std::cout << jointPoseData.size() << " joint poses are read" << std::endl;
    // fps
    double fps = 1000;
    if(argc >= 6){
        std::string fpsIn(argv[5]); // [1,1000]
        fps = std::stod(fpsIn);
        if(fps < 1){
            fps = 1;
        }else if(fps > 1000){
            fps = 1000;
        }
    }
    try
    {
        // Init. the robot
        franka::Robot robot(argv[1]);
        franka::Model model = robot.loadModel();
        std::cout << "This program will run the robot. Please keep the user stop button at hand!" << std::endl
            << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        setDefaultBehavior(robot);
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        // Run the Cartesin pose one by one
        for (unsigned int k = 0; k < jointPoseData.size(); k++)
        {
            // Joint pose [i]
            try
            {
                // Motion generator
                // Here we use the MotionGenerator in examples_common.cpp
                // For RT joint motion generator, please refer to kambuchaJointPoseRT.cpp
                std::array<double,7> q_goal;
                for (int j = 0; j < 7; j++)
                {
                    q_goal[j] = jointPoseData[k][j];
                    std::cout << q_goal[j] << ',';
                }
                std::cout << std::endl;
                double timer = 0.0;
                std::array<double,7> init_jp;
                unsigned int counter = 1;
                robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions{
                    // Joint pose motion plan with default impedance controller
                    timer += period.toSec();
                    if(timer == 0.0){
                        init_jp = state.q_d;        // The 1st joint pose command must be the current one
                    }
                    // S-spline interpolation
                    franka::JointPositions jointPose_c = {{
                            init_jp[0] + (q_goal[0] - init_jp[0])/2*(1-std::cos(M_PI_4*timer*speed)),
                            init_jp[1] + (q_goal[1] - init_jp[1])/2*(1-std::cos(M_PI_4*timer*speed)),
                            init_jp[2] + (q_goal[2] - init_jp[2])/2*(1-std::cos(M_PI_4*timer*speed)),
                            init_jp[3] + (q_goal[3] - init_jp[3])/2*(1-std::cos(M_PI_4*timer*speed)),
                            init_jp[4] + (q_goal[4] - init_jp[4])/2*(1-std::cos(M_PI_4*timer*speed)),
                            init_jp[5] + (q_goal[5] - init_jp[5])/2*(1-std::cos(M_PI_4*timer*speed)),
                            init_jp[6] + (q_goal[6] - init_jp[6])/2*(1-std::cos(M_PI_4*timer*speed))}};
                    if(timer >= 4.0/speed){
                        return franka::MotionFinished(jointPose_c);
                    }
                    if(counter >= std::ceil(1000/fps)){
                        // Read the required data
                        for (unsigned int i = 0; i < 7; i++)
                        {
                            fileTauJ << state.tau_J[i] << ',';
                            fileJP << state.q[i] << ',';
                        }
                        fileTauJ << std::endl;
                        fileJP << std::endl;
                        for (unsigned int i = 0; i < 16; i++)
                        {
                            fileOTEE << state.O_T_EE[i] << ',';
                        }
                        fileOTEE << std::endl;
                        for (unsigned int i = 0; i < 6; i++)
                        {
                            fileKFK << state.K_F_ext_hat_K[i] << ',';
                            fileOFK << state.O_F_ext_hat_K[i] << ',';
                        }
                        fileKFK << std::endl;
                        fileOFK << std::endl;
                        counter = 1;
                    }
                    counter++;
                    return jointPose_c;
                });
            }
            catch(const franka::ControlException& e)
            {
                std::cerr << e.what() << '\n';
                std::cout << "Running error recovery..." << std::endl;
                robot.automaticErrorRecovery();
            }
            std::cout << "Joint pose: " << k << " finished" << std::endl;
            // Read the robot state data by readOnce
            franka::RobotState robot_state = robot.readOnce();
            if(argc >= 5){
                // OTEE
                for (int i = 0; i < 16; i++)
                {
                    fileOTEE << robot_state.O_T_EE[i] << ',';
                }
                fileOTEE << std::endl;
                // tauJ
                for (int i = 0; i < 7; i++)
                {
                    fileTauJ << robot_state.tau_J[i] << ',';
                }
                fileTauJ << std::endl;
                // q
                for (int i = 0; i < 7; i++)
                {
                    fileJP << robot_state.q[i] << ',';
                }
                fileJP << std::endl;
                // OFK
                for (int i = 0; i < 6; i++)
                {
                    fileOFK << robot_state.O_F_ext_hat_K[i] << ',';
                }
                fileOFK << std::endl;
                // KFK
                for (int i = 0; i < 6; i++)
                {
                    fileKFK << robot_state.K_F_ext_hat_K[i] << ',';
                }
                fileKFK << std::endl;
            }
        }
        std::cout << "Motion finished!" << std::endl;
        // Never forget to close those files
        fileOTEE.close();
        fileTauJ.close();
        fileJP.close();
        fileKFK.close();
        fileOFK.close();
        if(argc < 5){
            // We do not need the robot state data. Delete them!
            std::remove(fileOTEEName.data());
            std::remove(fileTauJName.data());
            std::remove(fileJPName.data());
            std::remove(fileOFKName.data());
            std::remove(fileOFKName.data());
        }
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        // Never forget to close those files
        fileOTEE.close();
        fileTauJ.close();
        fileJP.close();
        fileKFK.close();
        fileOFK.close();
        return -1;
    }
    return 0;
}