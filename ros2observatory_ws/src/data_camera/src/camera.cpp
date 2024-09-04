#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <stdlib.h>

#include <gphoto2/gphoto2-camera.h>
#include <gphoto2-port-result.h>
#include <libgphoto2_functions.cpp>


//======= ROS2 includes===========//
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/msg/placeholder.hpp"
#include "interfaces/msg/event.hpp"
#include "interfaces/srv/int_status.hpp"
#include "interfaces/srv/int_request.hpp"
#include "interfaces/srv/string_status.hpp"
#include "interfaces/srv/string_request.hpp"
#include "interfaces/action/sequence.hpp"

using namespace std::chrono_literals;



// End of functions copied from libgphoto examples

class DataCamera : public rclcpp::Node
{
  public:
    //Constructor
    DataCamera()
    : Node("data_camera")
    {
      //=============== ROS Stuff==============//
      //Initialise publishers
      imagepublisher      = this->create_publisher<interfaces::msg::Placeholder>("data_stream", 10);
      eventpublisher      = this->create_publisher<interfaces::msg::Event>("camera_events", 10);
      //Initialise services
      batteryservice      = this->create_service<interfaces::srv::IntStatus>(
        "battery_status", std::bind(&DataCamera::battery_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      isogetservice          = this->create_service<interfaces::srv::IntStatus>(
        "get_iso", std::bind(&DataCamera::isoget_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      isosetservice          = this->create_service<interfaces::srv::IntRequest>(
        "set_iso", std::bind(&DataCamera::isoset_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      qualitygetservice = this->create_service<interfaces::srv::StringStatus>(
        "get_imgquality", std::bind(&DataCamera::qualityget_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      qualitysetservice = this->create_service<interfaces::srv::StringRequest>(
        "set_imgquality", std::bind(&DataCamera::qualityset_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      //Initialise actions
      sequenceaction      = rclcpp_action::create_server<interfaces::action::Sequence>(
        this, "sequence",
        std::bind(&DataCamera::sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DataCamera::sequence_cancel, this, std::placeholders::_1),
        std::bind(&DataCamera::sequence_accepted, this, std::placeholders::_1)
      );
      //Announce node start
      auto eventmessage = interfaces::msg::Event();
      eventmessage.event = "Camera Node starting";
      eventpublisher->publish(eventmessage);

      //============ gphoto2 stuff===============//
      //Connect to camera
      context = sample_create_context();
      isCameraConnected = detect_camera();
      if (isCameraConnected == false)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No Camera detected at startup");
      }

      //Initialise detection timer
      timercontext = 	gp_context_new(); //Basic context with no error reporting so the console doesnt get filled up
      detection_timer = this->create_wall_timer(
      500ms, std::bind(&DataCamera::detection_timer_callback, this));


    }

    //Destructor
    ~DataCamera(){
      //Close any connections to camera
      if(isCameraConnected){
        auto eventmessage = interfaces::msg::Event();
        eventmessage.event = "Camera Node shutting down";
        eventpublisher->publish(eventmessage);
        gp_camera_exit(camera, context);
        gp_camera_free(camera);
      }
    }
  private:
    // gphoto2 object handles
    Camera      *camera;
    int         ret;
    GPContext   *context;
    CameraText  text;

    GPContext   *timercontext;
    CameraWidget *rootwidget;
    
    bool isCameraConnected;
    
    
    //  HELPER FUNCTIONS
    
    bool detect_camera()
    {
	    /* This call will autodetect cameras, take the
	     * first one from the list and use it. It will ignore
	     * any others... See the *multi* examples on how to
	     * detect and use more than the first one.
	     */
      //Initialise camera variable
      gp_camera_new (&camera);
	    ret = gp_camera_init (camera, timercontext);
	    if (ret != GP_OK) {
		    gp_camera_unref(camera);
        return false;
	    }
      else {
        int ret = gp_camera_get_config (camera, &rootwidget, timercontext);
        if(ret == GP_OK){
          //Fetch camera make and model
          CameraWidget *widget;
          char *make;
          char *model;
          ret = gp_camera_get_single_config (camera, "manufacturer", &widget, timercontext);
          ret = gp_widget_get_value(widget,&make);
          ret = gp_camera_get_single_config (camera, "cameramodel", &widget, timercontext);
          ret = gp_widget_get_value(widget,&model);

          std::string makestring(make);
          std::string modelstring(model);
          std::string logstring = "Connected to camera: " + makestring + " " + modelstring;

          RCLCPP_INFO(this->get_logger(), logstring.c_str());
          auto eventmessage = interfaces::msg::Event();
          eventmessage.event = "Camera Connected: " + makestring + " " + modelstring;
          eventpublisher->publish(eventmessage);
        }
        else{
          RCLCPP_INFO(this->get_logger(),"Camera detected but unable to get configuration");
          gp_camera_unref(camera);
          gp_widget_unref(rootwidget);
        }

        return true;
      }
    }



    void capture_image()
    {
      auto imagemessage = interfaces::msg::Placeholder();
      imagemessage.image = "Pretend this is an image";
      auto eventmessage = interfaces::msg::Event();
      eventmessage.event = "Image Captured";
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << imagemessage.image);
      imagepublisher->publish(imagemessage);
      eventpublisher->publish(eventmessage);
    }

    //  PUBLISHERS, SUBSCRIBERS, SERVICES, ACTIONS and TIMERS
    rclcpp::Publisher<interfaces::msg::Placeholder>::SharedPtr imagepublisher;
    rclcpp::Publisher<interfaces::msg::Event>::SharedPtr eventpublisher;
    rclcpp::Service<interfaces::srv::IntStatus>::SharedPtr batteryservice;
    rclcpp::Service<interfaces::srv::IntStatus>::SharedPtr isogetservice;
    rclcpp::Service<interfaces::srv::IntRequest>::SharedPtr isosetservice;
    rclcpp::Service<interfaces::srv::StringStatus>::SharedPtr qualitygetservice;
    rclcpp::Service<interfaces::srv::StringRequest>::SharedPtr qualitysetservice;
    rclcpp_action::Server<interfaces::action::Sequence>::SharedPtr sequenceaction;
    rclcpp::TimerBase::SharedPtr detection_timer;

    //  SERVICE CALLBACKS
    void battery_callback(const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
      std::shared_ptr<interfaces::srv::IntStatus::Response> response)
    {
      std::string errorstring;
      RCLCPP_INFO(this->get_logger(), "Battery status requested");
        /*
        int ret;
        char  *batterylevel;
        ret = get_config_value_string(camera,"batterylevel",&batterylevel,context);
        if (ret < GP_OK){
          RCLCPP_INFO(this->get_logger(), "Error fetching battery level from camera");
          //Would be nice to print the gphoto error message to the ROS info system here
          response->value = 0;
          response->status = false;
          response->description = "Error fetching battery level from camera";
          RCLCPP_INFO(this->get_logger(), "Responding with fail status");          
        }
        else{
          std::string batterylevel_string(batterylevel);
          //Remove percentage sign from string
          batterylevel_string.pop_back();
          response->value = std::stoi(batterylevel_string);
          response->status = true;
          response->description = "";
          RCLCPP_INFO(this->get_logger(), "Responding with: %ld", (long int)response->value);
        }
        */
      int ret;
      CameraWidget *widget;
      char *batterylevel;
      std::string batterystring;
      if(!isCameraConnected){
        errorstring = "Camera not connected";
        goto batteryserviceerror;
      }
      ret = gp_camera_get_single_config (camera, "batterylevel", &widget, context);
      if(ret != GP_OK){
        std::string resultstring(gp_port_result_as_string(ret));
        errorstring = "Failed to get configuration widget: " + resultstring;
        goto batteryserviceerror;
      }
      ret = gp_widget_get_value(widget,&batterylevel);
      if(ret != GP_OK){
        std::string resultstring(gp_port_result_as_string(ret));
        errorstring = "Failed to get configuration value: " + resultstring;
        goto batteryserviceerror;
      }
      batterystring = std::string(batterylevel);
      batterystring.pop_back(); //Remove percent sign
      response->value = std::stoi(batterystring);
      response->status = true;
      response->description = "";
      RCLCPP_INFO(this->get_logger(), "Responding with %d", response->value);
      return;
      
      batteryserviceerror:
      response->value = 0;
      response->status = false;
      response->description = errorstring.c_str();
      RCLCPP_INFO(this->get_logger(), "Responding with fail status");
      return;
    }

    void isoget_callback(const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
      std::shared_ptr<interfaces::srv::IntStatus::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), "ISO setting requested");
      if(isCameraConnected){
        int ret;
        char  *isosetting;
        ret = get_config_value_string(camera,"iso",&isosetting,context);
        if (ret < GP_OK){
          RCLCPP_INFO(this->get_logger(), "Error fetching ISO setting from camera");
          //Would be nice to print the gphoto error message to the ROS info system here
          response->value = 0;
          response->status = false;
          response->description = "Error fetching ISO setting from camera";
          RCLCPP_INFO(this->get_logger(), "Responding with fail status");          
        }
        else{
          std::string isosetting_string(isosetting);
          std::cout << "ISO Setting: " << isosetting_string << std::endl;
          response->value = std::stoi(isosetting_string);
          response->status = true;
          response->description = "";
          RCLCPP_INFO(this->get_logger(), "Responding with: %ld", (long int)response->value);
        }

      }
      else{
        response->value = 0;
        response->status = false;
        response->description = "Camera not connected";
        RCLCPP_INFO(this->get_logger(), "Responding with fail status");
      }
    }

    void isoset_callback(const std::shared_ptr<interfaces::srv::IntRequest::Request> request,
      std::shared_ptr<interfaces::srv::IntRequest::Response> response)
    {
      //Here we use gphoto to request available iso values from the camera.
      //If the requested value is available, it is set and confirmation is given in the response.
      //Otherwise the response lists available values

      RCLCPP_INFO(this->get_logger(), "ISO setting request received: %ld", (long int) request->demand);
      
      if(isCameraConnected){
        std::vector<int> allowed_values;
        //Obtain list of allowed values
        CameraWidget		*widget = NULL, *child = NULL;
        int			ret;
        ret = gp_camera_get_config (camera, &widget, context);
	      if (ret < GP_OK) {
          //Return fail status here
	      }
        ret = _lookup_widget (widget, "iso", &child);
        //Again... check errors here
        int Nallowedvalues = gp_widget_count_choices(child);
        const char * choice;
        for (int i = 0; i < Nallowedvalues; i++){
          ret = gp_widget_get_choice(child,i,&choice);
          allowed_values.push_back(std::stoi(choice));
        }


        if(std::find(allowed_values.begin(), allowed_values.end(), request->demand) != allowed_values.end()){
          //If requested value is present in allowed_value
          std::string demandstring = std::to_string(request->demand);
          const char  *isosetting = demandstring.c_str();
          ret = set_config_value_string (camera, "iso", isosetting, context);
          if (ret == GP_OK){
            response->status = true;
            response->description = "Camera ISO set to " + std::to_string(request->demand);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ISO value changed to %ld", (long int) request->demand);
            auto eventmessage = interfaces::msg::Event();
            eventmessage.event = "ISO set to" + std::to_string(request->demand);
            eventpublisher->publish(eventmessage);
          }
          else{
            response->status = false;
            response->description = "Error changing setting on camera. Camera responded with GP Error code: " + std::to_string(ret);
            RCLCPP_INFO(this->get_logger(), "Error changing setting on camera.");
          }
        }
        else{
          std::string allowed_string = "[";
          for (int i = 0; i < allowed_values.size();i++) {
            if (i != (allowed_values.size() -1)) {
              allowed_string = allowed_string + std::to_string(allowed_values[i]) + ",";
            }
            else{
              allowed_string = allowed_string + std::to_string(allowed_values[i]) + "]";
            }
        
          }
          response->status = false;
          response->description = "Requested iso not available, allowed values are " + allowed_string;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Responding with fail status");
        
        
        }
      }
      else{
        response->status = false;
        response->description = "Camera not connected";
        RCLCPP_INFO(this->get_logger(), "Responding with fail status");
      }
    }

    void qualityget_callback(const std::shared_ptr<interfaces::srv::StringStatus::Request> request,
      std::shared_ptr<interfaces::srv::StringStatus::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), "Image quaility setting enquiry received");
      if(isCameraConnected){
        int ret;
        char  *imgquality;
        ret = get_config_value_string(camera,"imagequality",&imgquality,context);
        if (ret < GP_OK){
          RCLCPP_INFO(this->get_logger(), "Error fetching image quality setting from camera");
          //Would be nice to print the gphoto error message to the ROS info system here
          response->value = "";
          response->status = false;
          response->description = "Error fetching image quality setting from camera";
          RCLCPP_INFO(this->get_logger(), "Responding with fail status");          
        }
        else{
          std::string imgquality_string(imgquality);
          response->value = imgquality_string;
          response->status = true;
          response->description = "";
          RCLCPP_INFO(this->get_logger(), "Responding with: " );    //TODO: fix this so it reports the value
        }

      }
      else{
        response->value = "";
        response->status = false;
        response->description = "Camera not connected";
        RCLCPP_INFO(this->get_logger(), "Responding with fail status");
      }
    }

    void qualityset_callback(const std::shared_ptr<interfaces::srv::StringRequest::Request> request,
      std::shared_ptr<interfaces::srv::StringRequest::Response> response)
    {
      //Here we use gphoto to request available iso values from the camera.
      //If the requested value is available, it is set and confirmation is given in the response.
      //Otherwise the response lists available values

      RCLCPP_INFO(this->get_logger(), "Image quality setting request received: ");
      if(isCameraConnected){

        std::vector<std::string> allowed_values;
        allowed_values.push_back("LOW");
        allowed_values.push_back("MEDIUM");
        allowed_values.push_back("FINE");
        allowed_values.push_back("RAW");
        allowed_values.push_back("RAW+JPEG");

      
        if(std::find(allowed_values.begin(), allowed_values.end(), request->demand) != allowed_values.end()){
        //If requested value is present in allowed_value
          response->status = true;
          response->description = "Image quaility set to " + request->demand;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Image quality value changed to " );
          auto eventmessage = interfaces::msg::Event();
          eventmessage.event = "Image quality set to" + request->demand;
          eventpublisher->publish(eventmessage);
        }
        else{
          std::string allowed_string = "[" + allowed_values[0] + ",";
          for (int i = 1; i < allowed_values.size();i++) {
            if (i < (allowed_values.size() -1)) {
              allowed_string = allowed_string + allowed_values[i] + ",";
            }
            else{
              allowed_string = allowed_string + allowed_values[i] + "]";
            }
        
          }
          response->status = false;
          response->description = "Requested iso not available, allowed values are " + allowed_string;
          RCLCPP_INFO(this->get_logger(), "Responding with fail status");  
        
        }
      }
      else{
        response->status = false;
        response->description = "Camera not connected";
        RCLCPP_INFO(this->get_logger(), "Responding with fail status");
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

    //Timer Callbacks
    void detection_timer_callback()
    {
      if(!isCameraConnected){
        isCameraConnected = detect_camera();
      }
      else{
        //Check camera is still connected
        int retval = gp_camera_get_summary(camera,&text,timercontext);
        if (retval == GP_ERROR_IO_USB_FIND){
          isCameraConnected = false;
          RCLCPP_INFO(this->get_logger(), "Camera not found");
          auto eventmessage = interfaces::msg::Event();
          eventmessage.event = "Camera Disconnected";
          eventpublisher->publish(eventmessage);
        }
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCamera>());
  rclcpp::shutdown();
  return 0;
}