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
      imagepublisher   = this->create_publisher<interfaces::msg::Placeholder>("data_stream", 10);
      batteryservice  = this->create_service<interfaces::srv::BatteryRequest>(
        "battery_status", &battery_callback);

    }

  private:
    void capture_image()
    {
      auto message = interfaces::msg::Placeholder();
      message.image = "Here is an image";
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.image);
      imagepublisher->publish(message);
    }


    rclcpp::Publisher<interfaces::msg::Placeholder>::SharedPtr imagepublisher;
    rclcpp::Service<interfaces::srv::BatteryRequest>::SharedPtr batteryservice;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCamera>());
  rclcpp::shutdown();
  return 0;
}