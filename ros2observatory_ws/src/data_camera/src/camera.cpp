#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <string.h>
#include <stdlib.h>

#include <gphoto2/gphoto2.h>
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
      isCameraConnected = false;

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
    
    void connect_camera()
    {
	    /* This call will autodetect cameras, take the
	     * first one from the list and use it. It will ignore
	     * any others... See the *multi* examples on how to
	     * detect and use more than the first one.
	     */
      /*
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
      */
     //Initialise camera variable
      gp_camera_new(&camera);
      ret = gp_camera_init(camera, timercontext);
      if(ret != GP_OK){
        gp_camera_unref(camera);
        return;
      }
      isCameraConnected = true; //Need to set this here so the get_setting_value functions work... might want to change this in the future
      char *make;
      char *model;
      std::string errorstring;
      if(!get_setting_value(this,camera,timercontext,"manufacturer",&make,&errorstring)){
        RCLCPP_INFO_STREAM(this->get_logger(), "Camera detected but error fetching manufacturer: " << errorstring);
        gp_camera_unref(camera);
        isCameraConnected = false;
        return;
      }
      if(!get_setting_value(this,camera,timercontext,"cameramodel",&model,&errorstring)){
        RCLCPP_INFO_STREAM(this->get_logger(), "Camera detected but error fetching camera model: " << errorstring);
        gp_camera_unref(camera);
        isCameraConnected = false;
        return;
      }
      //TODO: Set storage location to match node parameter once it is implemented

      std::string makestring(make);
      std::string modelstring(model);
      auto eventmessage = interfaces::msg::Event();
      eventmessage.event = "Camera Connected: " + makestring + " " + modelstring;
      eventpublisher->publish(eventmessage);
      RCLCPP_INFO_STREAM(this->get_logger(), "Camera Connected: " << makestring << " " << modelstring);
      return;

    }
    void disconnect_camera(){
      gp_camera_unref(camera);
      auto eventmessage = interfaces::msg::Event();
      eventmessage.event = "Camera Disconnected";
      eventpublisher->publish(eventmessage);
      RCLCPP_INFO_STREAM(this->get_logger(), "Camera Disconnected");
      isCameraConnected = false;
      return;  
    }

    bool get_setting_value(DataCamera *node,Camera *camera, GPContext *context, char * key, char ** value, std::string *errorstring){
      int ret;
      CameraWidget *widget;
      char * val;
      if(!node->isCameraConnected){   //Check node is currently connected to a camera
        *errorstring = "Camera not connected";
        goto error;
      }
      ret = gp_camera_get_single_config (camera, key, &widget, context);    //Fetch the configuration widget that corresponds to the setting
      if(ret != GP_OK){
        *errorstring = "Failed to get configuration widget: " + std::string(gp_port_result_as_string(ret));
        goto error;
      }
      ret = gp_widget_get_value(widget,&val);   //Get the value associated with that configuration widget
      if(ret != GP_OK){
        *errorstring = "Failed to get configuration value: " + std::string(gp_port_result_as_string(ret));
        goto error;
      }
      //std::cout << val << std::endl;
      //Copy string to passed variable... not sure why this is needed, need to improve my understanding of both cstrings and pointers
      *value = strdup (val);
      return true;
      
      error:
      return false;
    }

    bool set_menu_setting_value(DataCamera *node,Camera *camera, GPContext *context, char * key, const char * demand, std::string *errorstring){
      int ret, Nchoices;
      CameraWidget *widget=NULL, *child = NULL;
      std::vector<std::string> allowed_values;
      std::string allowed_string;
      if(!node->isCameraConnected){ //Check node is currently connected to a camera
        *errorstring = "Camera not connected";
        goto seterror;
      }
      ret = gp_camera_get_config(camera,&widget,context); //Fetch the configuration widget
      if(ret != GP_OK){
        *errorstring = "Failed to get root configuration widget: " + std::string(gp_port_result_as_string(ret));
        goto seterror;
      }
      ret = _lookup_widget(widget,key,&child);
      if(ret != GP_OK){
        *errorstring = "Failed to get setting configuration widget: " + std::string(gp_port_result_as_string(ret));
        goto seterror;
      }
      //Build list of allowed values
      Nchoices = gp_widget_count_choices(child);
      const char * choice;
      for (int i = 0; i < Nchoices; i++){
        ret = gp_widget_get_choice(child,i,&choice);
        if (ret != GP_OK){
          *errorstring = "Failed to get configuration choice: " + std::string(gp_port_result_as_string(ret));
          goto seterror;
        }
        allowed_values.push_back(choice);
      }
      
      //Check if demand is present in allowed values
      if(!(std::find(allowed_values.begin(), allowed_values.end(),std::string(demand)) != allowed_values.end())){
        allowed_string = "[";
        for (int i = 0; i < allowed_values.size(); i++){
          if (i != (allowed_values.size()-1)){
            allowed_string = allowed_string+allowed_values[i] + ",";
          }
          else{
            allowed_string = allowed_string + allowed_values[i] + "]";
          }
        }
        *errorstring = "Allowed values are: " + allowed_string;
        goto seterror;
      }
      ret = gp_widget_set_value(child,demand); //Update widget with new value
      if(ret != GP_OK){
        *errorstring = "Error updating widget with new value: " + std::string(gp_port_result_as_string(ret));
        goto seterror;
      }
      /*
      ret = gp_camera_set_single_config(camera,key,widget,context);
      if(ret != GP_OK){
        ret = gp_camera_set_config(camera,widget,context);
        if(ret!=GP_OK){
          *errorstring = "Error uploading configuration widget to camera: " + std::string(gp_port_result_as_string(ret));
          goto seterror;
        }
      }
      */
      ret = gp_camera_set_config(camera,widget,context);
      if(ret!=GP_OK){
        *errorstring = "Error uploading configuration widget to camera: " + std::string(gp_port_result_as_string(ret));
        goto seterror;
      }
      return true;
      seterror:
      std::cout << "Error block entered" << std::endl;
      return false;
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

      //Start experimenting with how to take actual images. Once that is done we can figure out
      //how to package them into a ROS message.
      //
      // I see two functions in libgphoto for capturing an image. gp_camera_capture and gp_camera_capture_preview.
      //The documentation states that the first will capture an image on the camera at the path specified in its arguments
      //whereas the second does not store it on the camera and instead returns the image as a file.
      //The second one sounds like it could be good, as it prevents a second function call to get the file later, although
      //the first option is presumably more robust (could provide user with the option of wether to store to the SD card also ect.)
      //I will experiment with both, but I will start with gp_camera_capture_preview.
	    int ret;
	    CameraFile *file;
	    CameraFilePath camera_file_path;
	    FILE 	*f;
	    char	*data;
	    unsigned long size;

      //set capture target to camera ram...
      std::string errorstring;
      bool result = set_menu_setting_value(this,camera,context,"capturetarget","Internal RAM",&errorstring);
      if(!result){
        std::cout << errorstring << std::endl;
      }
	    printf("Capturing.\n");

	    /* NOP: This gets overridden in the library to /capt0000.jpg */
	    strcpy(camera_file_path.folder, "/");
	    strcpy(camera_file_path.name, "foo.jpg");

	    ret = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
	    if(ret != GP_OK){
        std::cout << "Error capturing image: " + std::string(gp_port_result_as_string(ret)) << std::endl;
      }

	    printf("Pathname on the camera: %s/%s\n", camera_file_path.folder, camera_file_path.name);

	    ret = gp_file_new(&file);
	    if(ret != GP_OK){
        std::cout << "Error initialising gp_file object: " + std::string(gp_port_result_as_string(ret)) << std::endl;
      }
	    ret = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name, GP_FILE_TYPE_NORMAL, file, context);
	    if(ret != GP_OK){
        std::cout << "Error fetching image from camera: " + std::string(gp_port_result_as_string(ret)) << std::endl;
      }

	    gp_file_get_data_and_size(file,(const char **)&data,&size);

	    printf("Deleting.\n");
	    ret = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name, context);
	    if(ret != GP_OK){
        std::cout << "Error deleting image from camera: " + std::string(gp_port_result_as_string(ret)) << std::endl;
      }

      f = fopen("test.jpg", "wb");
	    if (f) {
		    ret = fwrite (data, size, 1, f);
		    if (ret != (int)size) {
			    printf("  fwrite size %ld, written %d\n", size, ret);
		    }
		    fclose(f);
	    }
      else{
		    printf("  fopen test.jpg failed.\n");
      }
	    gp_file_free(file);
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
      char *batterylevel;
      if(get_setting_value(this,camera,context,"batterylevel",&batterylevel,&errorstring)){ 
        std::string batterystring = std::string(batterylevel);
        batterystring.pop_back(); //Remove percent sign
        response->value = std::stoi(batterystring);
        response->status = true;
        response->description = "";
        RCLCPP_INFO(this->get_logger(), "Responding with %d", response->value);
      }
      else{
        response->value = 0;
        response->status = false;
        response->description = errorstring.c_str();
        RCLCPP_INFO(this->get_logger(), "Responding with fail status");
      }
      return;
    }

    void isoget_callback(const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
      std::shared_ptr<interfaces::srv::IntStatus::Response> response)
    { 
      std::string errorstring;
      RCLCPP_INFO(this->get_logger(), "ISO setting requested");
      char *isosetting;
      if(get_setting_value(this,camera,context,"iso",&isosetting,&errorstring)){ 
        std::string isostring = std::string(isosetting);
        response->value = std::stoi(isostring);
        response->status = true;
        response->description = "";
        RCLCPP_INFO(this->get_logger(), "Responding with %d", response->value);
      }
      else{
        response->value = 0;
        response->status = false;
        response->description = errorstring.c_str();
        RCLCPP_INFO(this->get_logger(), "Responding with fail status");
      }
      return;
    }

    void isoset_callback(const std::shared_ptr<interfaces::srv::IntRequest::Request> request,
      std::shared_ptr<interfaces::srv::IntRequest::Response> response)
    {
      //Here we use gphoto to request available iso values from the camera.
      //If the requested value is available, it is set and confirmation is given in the response.
      //Otherwise the response lists available values

      RCLCPP_INFO(this->get_logger(), "ISO setting request received: %ld", (long int) request->demand);
      std::string errorstring;
      std::string demandstring = std::to_string(request->demand);
      const char* req = demandstring.c_str();
      if(set_menu_setting_value(this,camera,context,"iso",req,&errorstring)){
        response->status = true;
        response->description = "Camera ISO set to " + std::to_string(request->demand);
        RCLCPP_INFO(this->get_logger(), "ISO value changed to %ld", (long int) request->demand);
        auto eventmessage = interfaces::msg::Event();
        eventmessage.event = "ISO set to " + std::to_string(request->demand);
        eventpublisher->publish(eventmessage);
      }
      else{
        response->status = false;
        response->description = errorstring.c_str();
        RCLCPP_INFO(this->get_logger(), "Responding with fail status");
      }

      return;

    }

    void qualityget_callback(const std::shared_ptr<interfaces::srv::StringStatus::Request> request,
      std::shared_ptr<interfaces::srv::StringStatus::Response> response)
    {
      std::string errorstring;
      RCLCPP_INFO(this->get_logger(), "Current image quality setting requested");
      char *qualitysetting;
      if(get_setting_value(this,camera,context,"imagequality",&qualitysetting,&errorstring)){ 
        response->value = qualitysetting;
        response->status = true;
        response->description = "";
        std::string infomessage = "Responding with " + std::string(response->value);
        RCLCPP_INFO(this->get_logger(), infomessage.c_str());
      }
      else{
        response->value = "";
        response->status = false;
        response->description = errorstring.c_str();
        RCLCPP_INFO(this->get_logger(), "Responding with fail status");
      }
      return;
    }

    void qualityset_callback(const std::shared_ptr<interfaces::srv::StringRequest::Request> request,
      std::shared_ptr<interfaces::srv::StringRequest::Response> response)
    {
      //Here we use gphoto to request available iso values from the camera.
      //If the requested value is available, it is set and confirmation is given in the response.
      //Otherwise the response lists available values

      std::string logmessage = "Image quality setting request received: " + request->demand;
      RCLCPP_INFO(this->get_logger(), logmessage.c_str());
      std::string errorstring;
      std::string demandstring = request->demand;
      const char* req = demandstring.c_str();
      if(set_menu_setting_value(this,camera,context,"imagequality",req,&errorstring)){
        response->status = true;
        std::string decstring = "Camera image quality set to " + request->demand;
        response->description = decstring.c_str();
        logmessage = "Image quality setting changed to " + request->demand;
        RCLCPP_INFO(this->get_logger(), logmessage.c_str());
        auto eventmessage = interfaces::msg::Event();
        std::string eventstring = "Image quality set to " + request->demand;
        eventmessage.event = eventstring.c_str();
        eventpublisher->publish(eventmessage);
      }
      else{
        response->status = false;
        response->description = errorstring.c_str();
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
      if(!isCameraConnected){//TODO: once command queue is implemented, change this condition to !isCameraConnected && is queue empty.
        connect_camera();
      }
      else{
        //Check camera is still connected
        int ret = gp_camera_get_summary(camera,&text,timercontext);
        if (ret == GP_ERROR_IO_USB_FIND){
          disconnect_camera();
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