//kambuchaReadRT
//  Read the RobotState continuously
//  You can specify a tail name to identify the robot state file
//
//  Haopeng Hu
//  2020.06.29
//
//  kambuchaReadRT <fci-ip> tailName freqRead timeout

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/model.h>

int main(int argc, char** argv){
    if(argc < 2){
        // No fci-ip given
        std::cerr << "Usage: " << argv[0] << " <fci-ip> " << "tailName " << "freqRead " << "timeout " << std::endl;
        return -1;
    }
    std::cout << "Read the RobotState during demonstration" << std::endl << "Press Enter to continue ..." << std::endl;
    std::cin.ignore();
    // Init. file
    std::string tailName("tmp.csv");    // default tail name
    if(argc >= 3){
        tailName.assign(argv[2]);
        tailName.append(".csv");
    }
    // You may not need all of these data
    // Joint position
    std::string fileNameJP("RobotState_JP_");
    fileNameJP.append(tailName);
    std::ofstream fileOut_JP(fileNameJP,std::ios::out);
    // End effector pose frame in base frame
    std::string fileNameOTEE("RobotState_OTEE_");
    fileNameOTEE.append(tailName);
    std::ofstream fileOut_OTEE(fileNameOTEE,std::ios::out);
    // Stiffness pose frame in end effector frame
    std::string fileNameEETK("RobotState_EETK_");
    fileNameEETK.append(tailName);
    std::ofstream fileOut_EETK(fileNameEETK,std::ios::out);
    // Estimated wrench in stiffness frame
    std::string fileNameKFK("RobotState_KFK_");
    fileNameKFK.append(tailName);
    std::ofstream fileOut_KFK(fileNameKFK,std::ios::out);
    try
    {
        // Init. Robot
        franka::Robot robot(argv[1]);
        franka::Model model = robot.loadModel();
        // Arguments
        // Sample frequency
        size_t freqRead{1000};  // default: 1kHz
        if(argc >= 4){
            freqRead = std::atoi(argv[3]);  // You can assign it
            if(freqRead < 1){
                freqRead = 1;
            }else if(freqRead > 1000){
                freqRead = 1000;
            }
        }
        // Timeout
        size_t timeout{60}; // default: 1min
        if(argc >= 5){
            timeout = std::atoi(argv[4]);   // You can assign it
            if(timeout < 1){
                timeout = 1;
            }
        }
        // Counter
        size_t counter = 1;
        size_t timer = 0;
        std::cout << "Start reading robot state" << std::endl;
        std::cout << "fps: " << freqRead << std::endl
            << "timeout: " << timeout << std::endl
            << "data file: RobotState_DATA_" << tailName << std::endl;
        robot.read([&](const franka::RobotState& robot_state) -> bool{
            // Read what you want
            if(counter >= floor(1000/freqRead)){
                // Write to csv files
                // Joint position
                for (int i = 0; i < 7; i++)
                {
                    fileOut_JP << robot_state.q[i] << ',';
                }
                fileOut_JP << std::endl;
                // OTEE and EETK
                for (int i = 0; i < 16; i++)
                {
                    fileOut_OTEE << robot_state.O_T_EE[i] << ',';
                    fileOut_EETK << robot_state.EE_T_K[i] << ',';
                }
                fileOut_OTEE << std::endl;
                fileOut_EETK << std::endl;
                // KFK
                for (int i = 0; i < 6; i++)
                {
                    fileOut_KFK << robot_state.K_F_ext_hat_K[i] << ',';
                }
                fileOut_KFK << std::endl;
                // Reset counter
                counter = 1;
            }else{
                counter++;
            }
            timer++;
            return timer < timeout * 1000;
        });
        // Never forget to close these csv files
        fileOut_JP.close();
        fileOut_OTEE.close();
        fileOut_KFK.close();
        fileOut_EETK.close();
        return 0;
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        fileOut_JP.close();
        fileOut_OTEE.close();
        fileOut_KFK.close();
        fileOut_EETK.close();
        return -1;
    }
    
}