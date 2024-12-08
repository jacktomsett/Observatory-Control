#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <iostream>

//=====libgphoto2 includes========//
#include <gphoto2/gphoto2.h>
#include <gphoto2/gphoto2-camera.h>
#include <gphoto2-port-result.h>


//======= ROS2 includes===========//
#include "rclcpp/rclcpp.hpp"
//#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/msg/placeholder.hpp"
#include "interfaces/msg/data_image.hpp"
#include "interfaces/action/sequence.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "std_msgs/msg/string.hpp"



//========openCV includes=========//
#include <opencv2/core/core.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

//========other libraries=========//
#include "libraw/libraw.h"

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
    public:
        TestPublisher() : Node("test_publisher")
        {
            std::cout << "Node starting" << std::endl;
            imagepublisher = this->create_publisher<sensor_msgs::msg::Image>("data_stream", 10);
            
            
            // Let us create an image processor
            LibRaw iProcessor;
            // Open the file and read the metadata
            iProcessor.open_file("TestPictures/Sample.NEF");
            // Let us unpack the image
            iProcessor.unpack();
            // Process the image
            iProcessor.dcraw_process();
            // Store image in memory
            libraw_processed_image_t* image = iProcessor.dcraw_make_mem_image();


            cv::Mat imageMat;
            
            sensor_msgs::msg::Image imagemsg = cv_bridge::CvImage(std__msgs::Header(), "bgr8", imageMat).toImageMsg();

            
            imagepublisher->publish(imagemsg);

        }
        
    private:
    	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagepublisher;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TestPublisher>());
    rclcpp::shutdown();
}
