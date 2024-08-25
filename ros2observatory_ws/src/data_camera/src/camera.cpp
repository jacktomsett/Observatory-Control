#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/msg/placeholder.hpp"
#include "interfaces/srv/battery_request.hpp"
#include "interfaces/srv/confirmation.hpp"
#include "interfaces/srv/iso_request.hpp"
#include "interfaces/action/sequence.hpp"

using namespace std::chrono_literals;



class DataCamera : public rclcpp::Node
{
  public:
    DataCamera()
    : Node("data_camera")
    {
      //Initialise publishers
      imagepublisher      = this->create_publisher<interfaces::msg::Placeholder>("data_stream", 10);
      //Initialise services
      batteryservice      = this->create_service<interfaces::srv::BatteryRequest>(
        "battery_status", std::bind(&DataCamera::battery_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      isoservice          = this->create_service<interfaces::srv::IsoRequest>(
        "set_iso", std::bind(&DataCamera::iso_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      //Initialise actions
      sequenceaction      = rclcpp_action::create_server<interfaces::action::Sequence>(
        this, "sequence",
        std::bind(&DataCamera::sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DataCamera::sequence_cancel, this, std::placeholders::_1),
        std::bind(&DataCamera::sequence_accepted, this, std::placeholders::_1)
      );

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
    rclcpp::Service<interfaces::srv::BatteryRequest>::SharedPtr batteryservice;
    rclcpp::Service<interfaces::srv::IsoRequest>::SharedPtr isoservice;
    rclcpp_action::Server<interfaces::action::Sequence>::SharedPtr sequenceaction;

    //  SERVICE CALLBACKS
    void battery_callback(const std::shared_ptr<interfaces::srv::BatteryRequest::Request> request,
      std::shared_ptr<interfaces::srv::BatteryRequest::Response> response)
    {
      int battery = 69;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery status requested");
      response->percentage = battery;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Responding with: %ld", (long int)response->percentage);
    }

    void iso_callback(const std::shared_ptr<interfaces::srv::IsoRequest::Request> request,
      std::shared_ptr<interfaces::srv::IsoRequest::Response> response)
    {
      //Here we use gphoto to request available iso values from the camera.
      //If the requested value is available, it is set and confirmation is given in the response.
      //Otherwise the response lists available values

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ISO setting request received: %ld", (long int) request->iso);
      

      std::vector<int> allowed_values;
      allowed_values.push_back(100);
      allowed_values.push_back(200);
      allowed_values.push_back(400);
      allowed_values.push_back(600);
      allowed_values.push_back(800);
      allowed_values.push_back(1200);

      
      if(std::find(allowed_values.begin(), allowed_values.end(), request->iso) != allowed_values.end()){
      //If requested value is present in allowed_value
        response->confirmation = "Camera ISO set to " + std::to_string(request->iso);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ISO value changed to %ld", (long int) request->iso);
      }
      else{
        std::string allowed_string = "[";
        for (int i = 0; i < allowed_values.size();i++) {
          if (i != (allowed_string.size() -1)) {
            allowed_string = allowed_string + std::to_string(allowed_values[i]) + ",";
          }
          else{
            allowed_string = std::to_string(allowed_string[i]) + "]";
          }
        
        }
        response->confirmation = "Requested iso not available, allowed values are " + allowed_string;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Responding with fail status");
      }
    }

    //  ACTION CALLBACKS
    //====== Sequence Action =========
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


    void sequence_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goal_handle)
    {
      //This callback needs to finish quickly so it does not freeze up the system.
      //Interactions with gphoto will be done in a different thread.
      std::thread{std::bind(&DataCamera::sequence_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void sequence_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing sequence");
      rclcpp::Rate loop_rate(1);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<interfaces::action::Sequence::Feedback>();
      auto & current_image = feedback->current;
      auto exit_status = std::make_shared<interfaces::action::Sequence::Result>();

      for(int i = 1; (i < goal->length) && rclcpp::ok(); ++i)
      {
        //Check if there is a cancel request...
        if (goal_handle->is_canceling())
        {
          exit_status->confirmcomplete = "Sequence cancelled after " + feedback->current;
          goal_handle->canceled(exit_status);
          RCLCPP_INFO(this->get_logger(), "Sequence cancelled");
          return;
        }

        //Take image
        capture_image();

        //Update feedback
        current_image++;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Published Feedback");

        loop_rate.sleep();
      }

      //Check if goal is done
      if(rclcpp::ok())
      {
        exit_status->confirmcomplete = "Sequence complete";
        goal_handle->succeed(exit_status);
        RCLCPP_INFO(this->get_logger(), "Sequence succeeded");
      }

    }
    //===== Sequence Action end
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCamera>());
  rclcpp::shutdown();
  return 0;
}