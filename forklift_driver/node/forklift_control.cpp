#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <forklift_driver/msg/meteorcar.hpp>  

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

auto msg = "t : up (+z)\n"
          "b : down (-z)\n"
          "g : stop (z)\n"
          "q : speeds 1200\n"
          "a : speeds 2400\n"
          "z : speeds 3600\n";

// 定義保存終端設置的函數，返回終端設置
termios saveTerminalSettings() {
  termios settings;
  tcgetattr(STDIN_FILENO, &settings);
  return settings;
}

// 定義恢復終端設置的函數，使用傳入的設置進行恢復
void restoreTerminalSettings(termios old_settings) {
  #ifdef _WIN32
    return;
  #else
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
  #endif
}

// 定義獲取按鍵的函數
char getKey(termios settings) {
  #ifdef _WIN32
    // 在 Windows 上獲取按鍵
    return _getch();
  #else
    // 在 Linux 上獲取按鍵
    struct termios new_settings;
    tcgetattr(STDIN_FILENO, &new_settings);
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    char key;
    std::cin >> key;
    return key;
  #endif
}
char key;
termios settings;
class ForkLiftControl : public rclcpp::Node
{
  public:
    ForkLiftControl(): Node("teleop_forklift_keyboard")
    {
      try{
        // 保存終端設置
        settings = saveTerminalSettings();

        auto publisher_ = this->create_publisher<forklift_driver::msg::Meteorcar>("/cmd_fork", 5);
        auto vel = 1200.0f;
        
        auto message = forklift_driver::msg::Meteorcar();
        while (rclcpp::ok()){
          
          std::cout << msg << std::endl;
          std::cout << "-----------------------" << std::endl;
          key = getKey(settings);

          if(key == 'q' || key == 'Q'){
            vel = 1200.0f;
          }
          else if(key == 'a' || key == 'A'){
            vel = 2400.0f;
          }
          else if (key == 'z' || key == 'Z')
          {
            vel = 3600.0f;
          }

          if(key == 'g' || key == 'G'){
            message.fork_velocity = 0.0f;
            publisher_->publish(message);
          }
          else if(key == 't' || key == 'T'){
            message.fork_velocity = vel;
            publisher_->publish(message);
          }
          else if (key == 'b' || key == 'B')
          {
            message.fork_velocity = -vel;
            publisher_->publish(message);
          }
          std::cout << "fork_velocity" << vel << std::endl;

          // 恢復原來的終端設置
          restoreTerminalSettings(settings);
        }
      }
      catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        // 恢復原來的終端設置
        restoreTerminalSettings(settings);
      }
    }
  private:

    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<forklift_driver::msg::Meteorcar>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForkLiftControl>());

  rclcpp::shutdown();
  return 0;
}