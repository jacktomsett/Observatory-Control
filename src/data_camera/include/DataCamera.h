#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/msg/event.hpp"
#include "interfaces/srv/bool_status.hpp"
#include "interfaces/srv/bool_request.hpp"
#include "interfaces/srv/int_status.hpp"
#include "interfaces/srv/int_request.hpp"
#include "interfaces/srv/string_status.hpp"
#include "interfaces/srv/string_request.hpp"
#include "interfaces/action/sequence.hpp"
#include <gphoto2/gphoto2.h>
#include <thread>
#include <mutex>

class EventRequest; //forward declaration
class DataCamera: public rclcpp::Node
{
public:
  DataCamera();
  ~DataCamera();

  void insertEvent(std::shared_ptr < EventRequest >);
  bool get_setting_value(char *, char **, std::string *);
  bool set_menu_setting_value(char *, const char *, std::string *);
  bool change_toggle_setting_value(char *, bool, std::string *);
  bool capture_image();

  //TODO: Bad practice to have public class fields, should be replaced with getter and setters
  std::string currentSequenceId;
  int currentSequencePhotoNumber;
  int currentSequenceSuccesses;
  int currentSequenceFails;

private:
        //State tracking variables
  bool shutdownRequest;
  bool isCameraConnected;
  std::vector < std::shared_ptr < EventRequest >> eventQueue;
  std::shared_ptr < EventRequest > currentEvent;
  std::mutex queueLock;
  std::mutex currentEventLock;

        //libgphoto variables and functions
  Camera *cameraHandle;
  GPContext *context;           //Main context which reports to ROS info and error streams
  GPContext *emptyContext;          //Empty context with no error reporting. Used when no camera is connected and we are poling to detect one

  static void contextErrorFunction(GPContext *, const char *, void *);
  static void contextStatusFunction(GPContext *, const char *, void *);

  std::string errorstring;       //Holds last error from camera, to send back to service clients //FIXME: Not particularly robust, should be overhauled

        //Publishers, Subscribers, Services, Actions, Parameters
  rclcpp::Publisher < interfaces::msg::Event > ::SharedPtr eventpublisher;
  rclcpp::Service < interfaces::srv::IntStatus > ::SharedPtr batteryservice;
  rclcpp::Service < interfaces::srv::IntStatus > ::SharedPtr getisoservice;
  rclcpp::Service < interfaces::srv::IntRequest > ::SharedPtr setisoservice;
  rclcpp::Service < interfaces::srv::StringStatus > ::SharedPtr getqualservice;
  rclcpp::Service < interfaces::srv::StringRequest > ::SharedPtr setqualservice;
  rclcpp::Service < interfaces::srv::StringStatus > ::SharedPtr getfnumberservice;  
  rclcpp::Service < interfaces::srv::StringRequest > ::SharedPtr setfnumberservice; 
  rclcpp::Service < interfaces::srv::StringStatus > ::SharedPtr getexposureservice; 
  rclcpp::Service < interfaces::srv::StringRequest > ::SharedPtr setexposureservice;
  rclcpp::Service < interfaces::srv::StringStatus > ::SharedPtr getfocusmodeservice;
  rclcpp::Service < interfaces::srv::StringRequest > ::SharedPtr setfocusmodeservice; //FIXME: libgphoto issue. For some reason the focus mode setting is read only despite the camera being in manual mode
  rclcpp_action::Server < interfaces::action::Sequence > ::SharedPtr requestSequenceAction;


        //Camera Thread
  std::thread cameraThread;
  void cameraThreadFunction();

        //Helper functions
  void connectToCamera();
  void disconnectCamera();
  void checkCameraConnection();

        //Service Callbacks
  void battery_callback(
    const std::shared_ptr < interfaces::srv::IntStatus::Request >,
    std::shared_ptr < interfaces::srv::IntStatus::Response >);
  void getiso_callback(
    const std::shared_ptr < interfaces::srv::IntStatus::Request >,
    std::shared_ptr < interfaces::srv::IntStatus::Response >);
  void setiso_callback(
    const std::shared_ptr < interfaces::srv::IntRequest::Request >,
    std::shared_ptr < interfaces::srv::IntRequest::Response >);
  void getqual_callback(
    const std::shared_ptr < interfaces::srv::StringStatus::Request >,
    std::shared_ptr < interfaces::srv::StringStatus::Response >);
  void setqual_callback(
    const std::shared_ptr < interfaces::srv::StringRequest::Request >,
    std::shared_ptr < interfaces::srv::StringRequest::Response >);
   //TODO: It would be better if the following four services accepted and reported doubles rather than strings. The camera reports the value as strings so they would need to be formatted. This would require modifying the functions that communicate with the camera
  void getfnumber_callback(
    const std::shared_ptr < interfaces::srv::StringStatus::Request >,
    std::shared_ptr < interfaces::srv::StringStatus::Response >);
  void setfnumber_callback(
    const std::shared_ptr < interfaces::srv::StringRequest::Request >,
    std::shared_ptr < interfaces::srv::StringRequest::Response >);
  void getexposure_callback(
    const std::shared_ptr < interfaces::srv::StringStatus::Request >,
    std::shared_ptr < interfaces::srv::StringStatus::Response >);
  void setexposure_callback(
    const std::shared_ptr < interfaces::srv::StringRequest::Request >,
    std::shared_ptr < interfaces::srv::StringRequest::Response >);
  void getfocusmode_callback(
    const std::shared_ptr < interfaces::srv::StringStatus::Request >,
    std::shared_ptr < interfaces::srv::StringStatus::Response >);
  void setfocusmode_callback(
    const std::shared_ptr < interfaces::srv::StringRequest::Request >,
    std::shared_ptr < interfaces::srv::StringRequest::Response >);
  //TODO: The front end probably will want the ability to request possible options for a given service, rather than having to guess first and check the error description
  //TODO: In addition to above, probably want a service that fetches all configuration values simultaneously
  //TODO: Add service to enable/disable long exposure noise reduction
  //TODO: Add a service to enable/disable flash
  //TODO: Add a service for setting white balance (Does this matter if shooting in RAW?)
  //TODO: Add a service for disabling the assist light

        //Action Callbacks
  rclcpp_action::GoalResponse sequenceGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr < const interfaces::action::Sequence::Goal >);
  rclcpp_action::CancelResponse sequenceCancel(
    const std::shared_ptr <
    rclcpp_action::ServerGoalHandle < interfaces::action::Sequence >>);
  void sequence_accepted(
    const std::shared_ptr < rclcpp_action::ServerGoalHandle <
    interfaces::action::Sequence >>);

};
