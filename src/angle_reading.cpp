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

// 키보드 비동기 입력 관련 함수
static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard(){
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard(){
    tcsetattr(0, TCSANOW, &initial_settings);
}

int _kbhit(){
    unsigned char ch;
    int nread;
    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);

    if(nread == 1){
        peek_character = ch;
        return 1;
    }
    return 0;
}

int _getch(){
    char ch;
    if(peek_character != -1){
        ch = peek_character;
        peek_character = -1;
        return ch;
    }

    read(0,&ch,1);
    return ch;
}

int _putch(int c){
    putchar(c);
    fflush(stdout);
    return c;
}
// 키보드 비동기 입력 관련 함수 끝...


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
    init_keyboard();

    while(rclcpp::ok()){
        auto t2 = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);

        // 비동기 키보드 입력
        // h버튼 누르면 hold - 토크 있음
        // u버튼 누르면 unhold - 토크 없음
        if(_kbhit()){
            int ch = _getch();
            _putch(ch);
            if(ch == 'h'){
                std::cout << std::endl;
                std::cerr << "Try moving it by hand - should be holding position" << std::endl;
                drvr_->setManualModeAll(false, joint_num);
            }
            else if(ch == 'u'){
                std::cout << std::endl;
                std::cout << "Try moving it by hand - should be free to move" << std::endl;
                drvr_->setManualModeAll(true, joint_num);
            }
        }

        // time interval이 0.500초를 넘으면 한번씩 정보 reading
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
    }

    std::cout << "Bye" << std::endl;
    rclcpp::shutdown();
    return 0;
}
