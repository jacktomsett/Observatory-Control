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
        TestPublisher() : Node("test_publisher")
        {
            int ret;
            std::cout << "Node starting" << std::endl;
            imagepublisher = this->create_publisher<sensor_msgs::msg::Image>("data_stream", 10);
            std::cout << "Initialised publisher" << std::endl;
            
            // Let us create an image processor
            LibRaw iProcessor;
            std::cout << "Created LibRaw instance" << std::endl;
            // Open the file and read the metadata
            ret =  iProcessor.open_file("/home/jack/Observatory-Control/TestPictures/Sample.NEF");
            if(ret == 0){
                std::cout << "LibRaw opened the file Sample.NEF" << std::endl;
            } else{
                std::cout << "Error opening file: " << ret << std::endl;
            }
            // Let us unpack the image
            ret = iProcessor.unpack();
            if(ret == 0){
                std::cout << "LibRaw unpacked the file" << std::endl;
            } else{
                std::cout << "Error unpacking file: " << ret << std::endl;
            }
            /*
            ret = iProcessor.raw2image();
            if(ret == 0){
                std::cout << "LibRaw produced image from raw data" << std::endl;
            } else{
                std::cout << "Error producing image from raw data: " << ret << std::endl;
            }
            */

           //Examine image metadata
            std::cout << "Image width: " <<  iProcessor.imgdata.sizes.iwidth << std::endl;
            std::cout << "Image height: " << iProcessor.imgdata.sizes.iheight << std::endl;
            std::cout << "RAW width: " <<  iProcessor.imgdata.sizes.raw_width << std::endl;
            std::cout << "RAW height: " << iProcessor.imgdata.sizes.raw_height << std::endl;
            std::cout << "width: " <<  iProcessor.imgdata.sizes.width << std::endl;
            std::cout << "height: " << iProcessor.imgdata.sizes.height << std::endl;
            std::cout << "RAW data width: " <<  iProcessor.imgdata.sizes.raw_pitch << std::endl;
            std::cout << "RAW data bits per pixel: " << iProcessor.imgdata.color.raw_bps <<std::endl;
            std::cout << "Number of colours: " << iProcessor.imgdata.idata.colors << std::endl;
            std::cout << "Colour description: " << iProcessor.imgdata.idata.cdesc << std::endl;
            std::cout << "Camera: " << iProcessor.imgdata.idata.make << " " << iProcessor.imgdata.idata.model << std::endl;

            //Check data fields
            if (iProcessor.imgdata.rawdata.raw_alloc == NULL){
                std::cout << "void raw_alloc pointer is not assigned" << std::endl;
            } else{
                std::cout << "void raw_alloc pointer is assigned!" << std::endl;
            }
            if (iProcessor.imgdata.rawdata.raw_image == NULL){
                std::cout << "void raw_image pointer is not assigned" << std::endl;
            } else{
                std::cout << "void raw_image pointer is assigned!" << std::endl;
            }
            if (iProcessor.imgdata.rawdata.color3_image == NULL){
                std::cout << "void color3_image pointer is not assigned" << std::endl;
            } else{
                std::cout << "void color3_image pointer is assigned!" << std::endl;
            }
            if (iProcessor.imgdata.rawdata.color4_image == NULL){
                std::cout << "void color4_image pointer is not assigned" << std::endl;
            } else{
                std::cout << "void color4_image pointer is assigned!" << std::endl;
            }
            if (iProcessor.imgdata.rawdata.float_image == NULL){
                std::cout << "void float_image pointer is not assigned" << std::endl;
            } else{
                std::cout << "void float_image pointer is assigned!" << std::endl;
            }
            if (iProcessor.imgdata.rawdata.float3_image == NULL){
                std::cout << "void float3_image pointer is not assigned" << std::endl;
            } else{
                std::cout << "void float3_image pointer is assigned!" << std::endl;
            }
            if (iProcessor.imgdata.rawdata.float4_image == NULL){
                std::cout << "void float4_image pointer is not assigned" << std::endl;
            } else{
                std::cout << "void float4_image pointer is assigned!" << std::endl;
            }



            cv::Mat imageMat(iProcessor.imgdata.sizes.raw_height,iProcessor.imgdata.sizes.raw_width, CV_16UC1, iProcessor.imgdata.rawdata.raw_image,iProcessor.imgdata.sizes.raw_pitch);
            //cv::Mat imageMat;
            sensor_msgs::msg::Image::Ptr imagemsg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", imageMat).toImageMsg();

            
            imagepublisher->publish(*imagemsg);
            std::cout << "Message published" << std::endl;

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
