#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/placeholder.hpp"
#include "interfaces/srv/battery_request.hpp"
#include "interfaces/srv/confirmation.hpp"

using namespace std::chrono_literals;



class DataCamera : public rclcpp::Node
{
  public:
    DataCamera()
    : Node("data_camera")
    {
      imagepublisher      = this->create_publisher<interfaces::msg::Placeholder>("data_stream", 10);
      startcaptureservice = this->create_service<interfaces::srv::Confirmation>(
        "start_capture", std::bind(&DataCamera::startcapture_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      batteryservice      = this->create_service<interfaces::srv::BatteryRequest>(
        "battery_status", std::bind(&DataCamera::battery_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      sequenceaction      = rclcpp_action::create_server<interfaces::action::Sequence>(
        this, "sequence",
        std::bind(&DataCamera::sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DataCamera::sequence_cancel, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DataCamera::sequence_accepted, this, std::placeholders::_1, std::placeholders::_2)
      )

    }

  private:
    //  HELPER FUNCTIONS
    
    void capture_image()
    {
      auto message = interfaces::msg::Placeholder();
      message.image = "Here is an image";
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.image);
      imagepublisher->publish(message);
    }

    //  PUBLISHERS, SUBSCRIBERS SERVICES AND ACTIONS
    rclcpp::Publisher<interfaces::msg::Placeholder>::SharedPtr imagepublisher;
    rclcpp::Service<interfaces::srv::Confirmation>::SharedPtr startcaptureservice;
    rclcpp::Service<interfaces::srv::BatteryRequest>::SharedPtr batteryservice;
    rclcpp_action::Server<interfaces::action::Sequence>::SharedPtr sequenceaction;

    //  SERVICE CALLBACKS
    void startcapture_callback(const std::shared_ptr<interfaces::srv::Confirmation::Request> request,
      std::shared_ptr<interfaces::srv::Confirmation::Response> response)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting capture");
      response->confirmation = "Starting capture...";
      for (int i = 0; i < 5; i++){
        this->capture_image();
      }
    }


    void battery_callback(const std::shared_ptr<interfaces::srv::BatteryRequest::Request> request,
      std::shared_ptr<interfaces::srv::BatteryRequest::Response> response)
    {
      int battery = 69;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery status requested");
      response->percentage = battery;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Responding with: %ld", (long int)response->percentage);
    }

    //  ACTION CALLBACKS
    void sequence_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goal_handle)
    {
      //This callback needs to finish quickly so it does not freeze up the system.
      //Interactions with gphoto will be done in a different thread.
      std::thread{std::bind(&DataCamera::capture_sequence, this, std::placeholders::-1),goal_handle}.detach();
    }

    rclcpp_action::GoalResponse sequence_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const interfaces::action::Sequence::Goal> goal
    )
    {
      RCLCPP_INFO(this->get_logger(), "Received request for sequence of %d", goal->length);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }//This has been copied from tutorial, and just accepts all goals. Obviously the camera cannot take two sequences
    // simulataneously. We need to either cancel the current goal and start a new once if one exists, or just refuse a
    // a new goal if a current goal exists

    rclcpp_action::CancelResponse sequence_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goal_handle
    )
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel sequence");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }//Accoring to tutorial, this just tells the client that the cancellation has been accepted. Presumable to actually
    //cancel something you would need to do something with the goal_handle parameter.

    void capture_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing sequence");
      rclcpp:Rate loop_rate(1);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<interfaces::action::Sequence::Feedback>();
      
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCamera>());
  rclcpp::shutdown();
  return 0;
}