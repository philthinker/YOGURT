//kambuchaCarteOEPose
//  Move to the given Cartesian pose w.r.t. base frame or end effector frame
//  You can specify a file name to assign the goal pose
//  
//  Haopeng Hu
//  2020.06.29
//
//  kambuchaCarteOEPose <fci-ip> O/E fileName

#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/model.h>

#include <Eigen/Dense>

int main(int argc, char** argv){
    if(argc < 4){
        std::cerr << "Usage: " << argv[0] << "<fci-ip> " << "O/E " << "fileName" << std::endl;
        return -1;
    }
    // Read frame and goal pose
    const std::string FRAME(argv[2]);   // frame
    std::string fileName(argv[3]);
    fileName.append(".csv");            // fileName.csv
    std::ifstream fileIn(fileName,std::ios::in);
    if(fileIn.fail()){
        std::cerr << "File: " << argv[3] << ".csv not found" << std::endl;
        return -1;
    }
    std::vector<std::vector<double>> cartePoseData; // Store the Cartesian pose data
    std::string line;                               // Store one line of data for file reading
    std::vector<double> cartePoseDataTmp;           // Store one Cartesian pose data
    while (std::getline(fileIn,line) && fileIn.good())
    {
        std::istringstream dataIn(line);
        std::string dataTmp;
        while (std::getline(dataIn,dataTmp,','))
        {
            cartePoseDataTmp.push_back(std::stod(dataTmp));
        }
        cartePoseData.push_back(cartePoseDataTmp);
    }
    std::cout << cartePoseData.size() << " Cartesian poses in " << FRAME << " frame are read" << std::endl;
    try
    {
        // Init. the robot
        franka::Robot robot(argv[1]);
        franka::Model model = robot.loadModel();
        std::cout << "This program will run the robot. Please keep the user stop button at hand!" << std::endl
            << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        // Torque controller
        auto controllerTorque = [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques{
            // Impedance control law
            franka::Torques tau_c({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
            return tau_c;
        };
        // Run the Cartesin pose one by one
        for (int i = 0; i < cartePoseData.size(); i++)
        {
            // Cartesian pose [i]
            try
            {
                // Motion generator
                double timer = 0.0;
                robot.control(controllerTorque, [&FRAME,&cartePoseData,&timer,i](const franka::RobotState state, franka::Duration period) 
                    -> franka::CartesianPose{
                    // Position and Quaternion interpolation
                    timer = period.toSec();
                    if(timer == 0.0){
                        // The first pose must be the initial pose
                    }
                });
            }
            catch(const franka::ControlException& e)
            {
                std::cerr << e.what() << '\n';
                std::cout << "Running error recovery..." << std::endl;
                robot.automaticErrorRecovery();
            }
            std::cout << "Cartesian pose: " << i << "finished" << std::endl;
        }
        std::cout << "Motion finished!" << std::endl;
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    return 0;
}