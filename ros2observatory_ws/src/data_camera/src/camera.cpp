#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/placeholder.hpp"
#include "interfaces/srv/battery_request.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
    void battery_callback(const std::shared_ptr<interfaces::srv::BatteryRequest::Request> request,
      std::shared_ptr<interfaces::srv::BatteryRequest::Response> response)
    {
      int battery = 69;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery status requested");
      response->percentage = battery;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Responding with: %ld", (long int)response->percentage);
    }


class DataCamera : public rclcpp::Node
{
  public:
    DataCamera()
    : Node("data_camera")
    {
      datapublisher   = this->create_publisher<interfaces::msg::Placeholder>("data_stream", 10);
      batteryservice  = this->create_service<interfaces::srv::BatteryRequest>(
        "battery_status", &battery_callback);
      timer_ = this->create_wall_timer(
        2000ms, std::bind(&DataCamera::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = interfaces::msg::Placeholder();
      message.image = "Published image";
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.image);
      datapublisher->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::Placeholder>::SharedPtr datapublisher;
    rclcpp::Service<interfaces::srv::BatteryRequest>::SharedPtr batteryservice;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCamera>());
  rclcpp::shutdown();
  return 0;
}