//kambuchaZeroGravity
//  Just used to test the default impedance controller
//  You can specify a file to set the impedance param.
//
//  Haopeng Hu
//  2020.06.30
//
//  kambuchaZeroGravity <fci-ip> J/C fileInName fileOutName fps

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cmath>

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>

int main(int argc, char** argv){
    if(argc < 6){
        std::cerr << "Usage: " << argv[0] << " <fci-ip>" << " J/C " << " fileInName " << " fileOutName " << " fps " << std::endl;
        return -1;
    }
    // Joint impedance or Cartesian impedance
    std::string impedFlag(argv[2]);
    // Impedance param.
    //  You can only specify ONE param., and the others will be discarded.
    std::string fileInName(argv[3]);
    std::ifstream fileIn(fileInName.append(".csv"),std::ios::in);
    if(fileIn.fail()){
        std::cerr << "File: " << argv[3] << ".csv not found" << std::endl;
        return -1;
    }
    // Default param.
    std::array<double,7> impedJ({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    std::array<double,6> impedC({{3000, 3000, 3000, 300, 300, 300}});
    // Data read
    std::string line;
    unsigned int k = 0;
    if(impedFlag == "C"){
        // Cartesian impedance param.
        while (std::getline(fileIn,line,',') && fileIn.good() && k < 6)
        {
            // We only care about the first line
            impedC[k] = std::stod(line);
            k++;
        }
    }else if(impedFlag == "J"){
        // Joint impedance param.
        while (std::getline(fileIn,line,',') && fileIn.good() && k < 7)
        {
            // We only care about the first line
            impedJ[k] = std::stod(line);
            k++;
        }
    }else
    {
        std::cerr << "You can only specify J or C" << std::endl;
        return -1;
    }
    std::cout << "Joint impedance : ";
    for (unsigned int i = 0; i < 7; i++)
    {
        std::cout << impedJ[i] << ',';
    }
    std::cout << std::endl << "Cartesian impedance : ";
    for (unsigned int i = 0; i < 6; i++)
    {
        std::cout << impedC[i] << ',';
    }
    std::cout << std::endl;
    // Init. file out
    std::string fileOutName(argv[4]);
    std::ofstream fileOut_JP(fileOutName.append("_JP.csv"), std::ios::out);
    fileOutName.assign(argv[4]);
    std::ofstream fileOut_OTEE(fileOutName.append("_OTEE.csv"), std::ios::out);
    fileOutName.assign(argv[4]);
    std::ofstream fileOut_OFK(fileOutName.append("_OFK.csv"), std::ios::out);
    fileOutName.assign(argv[4]);
    std::ofstream fileOut_KFK(fileOutName.append("_KFK.csv"), std::ios::out);
    fileOutName.assign(argv[4]);
    std::ofstream fileOut_tauJ(fileOutName.append("_tauJ.csv"), std::ios::out);
    fileOutName.assign(argv[4]);
    std::ofstream fileOut_cori(fileOutName.append("_cori.csv"), std::ios::out);
    // fps
    std::string fpsIn(argv[5]);
    double fps = std::stod(fpsIn);  // [1,1000]
    if(fps < 1){
        fps = 1;
    }else if(fps > 1000){
        fps = 1000;
    }
    // Init. robot
    franka::Robot robot(argv[1]);
    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    franka::Model model = robot.loadModel();
    // Start zero gravity controll
    try
    {
        std::cout << "Start default zero gravity control" << std::endl;
        unsigned int counter = 1;
        robot.control([&](const franka::RobotState& state, franka::Duration) -> franka::Torques{
            // Note that this program will nevert exit automatically
            std::array<double,7> torque({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
            // Read the data required
            if(counter >= std::ceil(1000/fps)){
                for (unsigned int i = 0; i < 7; i++)
                {
                    fileOut_JP << state.q[i] << ',';
                    fileOut_tauJ << state.tau_J[i] << ',';
                }
                fileOut_JP << std::endl;
                fileOut_tauJ << std::endl;
                for (unsigned int i = 0; i < 6; i++)
                {
                    fileOut_KFK << state.K_F_ext_hat_K[i] << ',';
                    fileOut_OFK << state.O_F_ext_hat_K[i] << ',';
                }
                fileOut_KFK << std::endl;
                fileOut_OFK << std::endl;
                for (unsigned int i = 0; i < 16; i++)
                {
                    fileOut_OTEE << state.O_T_EE[i] << ',';
                }
                fileOut_OTEE << std::endl;
                counter = 1;
            }
            counter++;
            return torque;
        });
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        // Never forget to close those files
        fileOut_JP.close();
        fileOut_OTEE.close();
        fileOut_OFK.close();
        fileOut_KFK.close();
        fileOut_tauJ.close();
        fileOut_cori.close();
        return -1;
    }
    // Never forget to close those files
    fileOut_JP.close();
    fileOut_OTEE.close();
    fileOut_OFK.close();
    fileOut_KFK.close();
    fileOut_tauJ.close();
    fileOut_cori.close();
    return 0;
}