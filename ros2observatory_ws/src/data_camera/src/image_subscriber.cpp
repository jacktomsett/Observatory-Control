#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/msg/placeholder.hpp"
#include "interfaces/msg/data_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>

//========openCV includes=========//
#include <opencv2/core/core.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>


using namespace std::chrono_literals;

class DataSubscriber : public rclcpp::Node
{
    public:
        DataSubscriber()
        : Node("data_subscriber")
        {
            count = 0;
            datafeed = this->create_subscription<sensor_msgs::msg::Image>(
                "data_stream",10,std::bind(&DataSubscriber::image_callback,this,std::placeholders::_1));
        }

    private:
        int count;  //Keep track of how many images we have. This whole node is just to check the
                    //images are transporting properly so I am not going  to worry about padding it
                    //or making it sophisticated. Eventually I will add a sequence name and photo number
                    //to the message making this unneccessary.
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr datafeed;
        void image_callback(const sensor_msgs::msg::Image & msg)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Data image received");
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            std::string filename = std::string("image")+std::to_string(count)+std::string(".jpg");
            cv::imwrite(filename,cv_ptr->image);
            count++;
            RCLCPP_INFO_STREAM(this->get_logger(),"Saved to " << filename);
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataSubscriber>());
    rclcpp::shutdown();
    return 0;
}