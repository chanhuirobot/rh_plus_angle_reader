#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <math.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"

#include "rh_plus_angle_reader/anglereader_serial.hpp"
#include "rh_plus_angle_reader/anglereader_drvr.hpp"

const std::string SERIAL_DEV = "/dev/ttyUSB0";

// 동기 방식 키보드 입력. 비동기 방식으로 해야 될듯.
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

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
    std::cout << "Type Number of Joint, and Press enter. >>> ";
    std::cin >> joint_num;
    std::cout << "-----------------------------------------------" << std::endl << std::endl;

    std::cout << "Try moving it by hand - should be free to move" << std::endl;
    drvr_->setManualModeAll(true, joint_num);

    auto t1 = std::chrono::high_resolution_clock::now();
    char input;

    while(rclcpp::ok()){
        auto t2 = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        input = getch();

        if(time_span.count() > 0.500){
            for(int i = 0; i < joint_num; i++){
                uint16_t pos;
                double rad;
                int degree;

                drvr_->getJointPosition(i+1, pos);
                rad = double(((pos - 500) / 1000.0) * (240.0 / 180.0) * M_PI);
                degree = int(((pos - 500) / 1000.0) * 240.0);

                std::cout << std::fixed;
                std::cout.precision(3);
                std::cout << "Joint : " << i+1 << "\tPos : " << pos << "\tRad : " << rad << "\tDegree : " << degree << std::endl;
            }
            std::cout << "-----------------------------------------------------------------------------------" << std::endl;
            t1 = std::chrono::high_resolution_clock::now(); // t1 업데이트
        }

        std::cout << "input : " << input << std::endl;
    }

    std::cout << "Bye" << std::endl;
    rclcpp::shutdown();
    return 0;
}
