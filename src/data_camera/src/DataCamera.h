#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/event.hpp"
#include "interfaces/srv/int_status.hpp"
#include <gphoto2/gphoto2.h>
#include <thread>
#include <mutex>

class EventRequest; //forward declaration
class DataCamera : public rclcpp::Node
{
    public:
        DataCamera();
        ~DataCamera();

        bool get_setting_value(char*, char**);

    private:
        //State tracking variables
        bool shutdownRequest;
        bool isCameraConnected;
        std::vector<EventRequest*> eventQueue;
        std::mutex queueLock;

        //libgphoto variables and functions
        Camera *cameraHandle;
        GPContext *context;     //Main context which reports to ROS info and error streams
        GPContext *emptyContext;    //Empty context with no error reporting. Used when no camera is connected and we are poling to detect one

        static void contextErrorFunction(GPContext*, const char*, void*);
        static void contextStatusFunction(GPContext*, const char*, void*);

        //Publishers, Subscribers, Services, Actions, Parameters
        rclcpp::Publisher<interfaces::msg::Event>::SharedPtr eventpublisher;
        rclcpp::Service<interfaces::srv::IntStatus>::SharedPtr batteryservice;

        //Camera Thread
        std::thread cameraThread;
        void cameraThreadFunction();

        //Helper functions
        void connectToCamera();
        void disconnectCamera();
        void checkCameraConnection();

        //Service Callbacks
        void battery_callback(const std::shared_ptr<interfaces::srv::IntStatus::Request>, std::shared_ptr<interfaces::srv::IntStatus::Response>);

};