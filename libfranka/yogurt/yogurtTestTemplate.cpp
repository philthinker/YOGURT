// yogurtTestTemplate
//
// Haopeng Hu
// 2020.05.29
// All rights reserved

// System 
#include<iostream>
#include<cmath>


int main(int argc, char** argv){
    // Check arguments
    // The fci-ip address always comes frist.
    if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " 172.16.0.2" << std::endl;
    return -1;
    try
    {
        // Wait for the keyboard command to run the program
        std::cout << "Take it easy. This is just a test!" << std::endl
                << "Press Enter to continue. Good luck!" << std::endl;
        std::cin.ignore();
        // Control strategy
        // Policy parameters
        
        // Control loop
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    return 0;
}
