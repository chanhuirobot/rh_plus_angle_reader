#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "rh_plus_angle_reader/anglereader_serial.hpp"
#include "rh_plus_angle_reader/anglereader_drvr.hpp"

const std::string SERIAL_DEV = "/dev/ttyUSB0";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::unique_ptr<anglereader::anglereader_drvr> drvr_ = std::make_unique<anglereader::anglereader_serial>();
    if (!drvr_->open(SERIAL_DEV))
    {
        std::cerr << "Failed to open driver";
        return false;
    }

    int joint_num;
    std::cout << "Type Number of Joint, and Press enter. >>> " << std::endl;
    std::cin >> joint_num;
    std::cout << "-----------------------------------------------" << std::endl << std::endl;

    std::cout << "Try moving it by hand - should be free to move" << std::endl;
    drvr_->setManualModeAll(true, 1);

    while(true){
        for(int i = 0; i < joint_num; i++){
            uint16_t pos;
            double rad;
            int degree;
            drvr_->getJointPosition(i, pos);

            std::cout << "Try moving it by hand - should be free to move" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Bye" << std::endl;
    rclcpp::shutdown();
    return 0;
}
