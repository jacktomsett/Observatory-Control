#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/msg/placeholder.hpp"
#include "interfaces/msg/data_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>

//========openCV includes=========//
#include <opencv2/core/core.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/highgui/highgui.hpp>


using namespace std::chrono_literals;

class DataSubscriber : public rclcpp::Node
{
    public:
        DataSubscriber()
        : Node("data_subscriber")
        {
            datafeed = this->create_subscription<interfaces::msg::DataImage>(
                "data_stream",10,std::bind(&DataSubscriber::image_callback,this,std::placeholders::_1));
        }

    private:
        rclcpp::Subscription<interfaces::msg::DataImage>::SharedPtr datafeed;
        void image_callback(const interfaces::msg::DataImage & msg)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Data image received");
            cv_bridge::CvImagePtr cv_ptr;
            //cv_ptr = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::BGR8);
            cv_ptr = cv_bridge::toCvCopy(msg.image);
            std::string filename = std::string(msg.sequencename)+std::to_string(msg.sequencenumber)+std::string(".jpg");
            cv::imwrite(filename,cv_ptr->image);
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