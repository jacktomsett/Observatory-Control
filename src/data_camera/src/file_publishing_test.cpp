#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

//=====libgphoto2 includes========//
#include <gphoto2/gphoto2.h>
#include <gphoto2/gphoto2-camera.h>
#include <gphoto2-port-result.h>


//======= ROS2 includes===========//
#include "rclcpp/rclcpp.hpp"
//#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/msg/placeholder.hpp"
#include "interfaces/msg/data_file.hpp"
#include "interfaces/action/sequence.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "std_msgs/msg/string.hpp"



//========openCV includes=========//
#include <opencv2/core/core.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/highgui/highgui.hpp>
//#include <image_transport/image_transport.hpp>

//========other libraries=========//
#include "libraw/libraw.h"

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
    public:
        TestPublisher() : Node("test_file_publisher")
        {
            int ret;
            std::cout << "Node starting" << std::endl;
            filepublisher = this->create_publisher<interfaces::msg::DataFile>("data_stream", 10);
            std::cout << "Initialised publisher" << std::endl;

            std::ifstream file;
            file.open("/home/jack/Observatory-Control/TestPictures/Sample.NEF");
            
           
            imagepublisher->publish(*filemsg);
            std::cout << "Message published" << std::endl;

        }
        
    private:
    	rclcpp::Publisher<interfaces::msg::DataFile>::SharedPtr filepublisher;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TestPublisher>());
    rclcpp::shutdown();
}
