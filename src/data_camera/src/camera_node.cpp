#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include "unistd.h"

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/event.hpp"
#include "interfaces/srv/int_status.hpp"

std::mutex queueLock;


class EventRequest
{
  public:
    EventRequest(int p, std::string name, void (*func)(EventRequest*) ){
      priority = p;
      testname = name;
      complete = false;
      status = false;
      eventFunctionPtr = func;
    }

    int priority;
    bool complete;
    bool status;
    std::string testname;
    std::string result;
    void execute()
    {
      std::cout << "About to hand over to event function pointer..." << std::endl;
      eventFunctionPtr(this);
      complete = true;
    }

  protected:
    void (*eventFunctionPtr)(EventRequest*);

};


void dummyRequestFunction(EventRequest* event)
{
  std::cout << "Entered dummyRequestedFunction" << std::endl;
  std::cout << "Event priority: " << event->priority << std::endl;
  std::cout << "Event name    : " << event->testname << std::endl;

  event->result = "10000000%";
  event->status = true;
  //Add sleep here to simulate the time that service callbacks might have to wait for
  usleep(5000);
  return;
};

class DataCamera : public rclcpp::Node
{
  public:
    DataCamera()
    : Node("data_camera")
    {
      shutdownRequest = false;
      batteryservice = this->create_service<interfaces::srv::IntStatus>(
        "battery_status", std::bind(&DataCamera::battery_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      cameraThread = std::thread(&DataCamera::cameraThreadFunction,this);
    }
    ~DataCamera()
    {
      std::cout << "Camera node shutting down..." << std::endl;
      shutdownRequest = true;
      cameraThread.join();
    }

  private:
    bool shutdownRequest;
    std::vector<EventRequest*> eventQueue;
    
    rclcpp::Service<interfaces::srv::IntStatus>::SharedPtr batteryservice;

    std::thread cameraThread;
    void cameraThreadFunction()
    {
      //Variable to hold current event
      EventRequest* currentEvent;


      while(shutdownRequest == false)
      {
        if( eventQueue.size() == 0 )
        {
          //Run keep alive command to check camera is still connected
          keepAlive();
        }
        else
        {
          //Create local copy of first event in queue
          queueLock.lock();
          currentEvent = eventQueue[0];
          eventQueue.erase(eventQueue.begin());
          queueLock.unlock();

          //Perform event function
          currentEvent->execute();

        }
      }
    }

    void keepAlive()
    {
      //TODO: implement camera connection checks here
      std::cout << "Event queue empty... polling camera" << std::endl;
      usleep(500000);
      
    }

    void battery_callback(const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
      std::shared_ptr<interfaces::srv::IntStatus::Response> response)
    {
      /*Dummy callback for now just to test adding to event queue from a ros service request
          and waiting for request to complete before reporting back. Will eventually need to
          implement inserting based on priority and adding a timeout before reporting back*/
      
      //Create event
      EventRequest event(1,"battery request",dummyRequestFunction);

      //Insert event request into queue
      //TODO: Will probably factor this out into its own function that can be shared amongst callbacks
      queueLock.lock();
      eventQueue.push_back(&event);
      queueLock.unlock();

      while (event.complete == false){}
      if (event.status == true)
      {
        event.result.pop_back(); //Remove percent sign
        response->value = std::stoi(event.result);
        response->status = true;
        response->description = "";
        //RCLCPP_INFO_STREAM(this, "Responding with " << response->value);
      }
      else
      {
        response->value = 0;
        response->status = false;
        response->description = "Need to implement error string here";
        //RCLCPP_INFO_STREAM(this, "Responding with fail status");
      }
      
    }
  };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCamera>());
  rclcpp::shutdown();

  return 0;
}
