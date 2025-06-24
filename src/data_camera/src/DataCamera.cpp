#include "DataCamera.h"
#include "EventsClasses.h"

DataCamera::DataCamera() : Node("data_camera")
{
    //Initialise variables
    shutdownRequest = false;
    isCameraConnected = false;

    //Initialise libgphoto contexts
    context = gp_context_new();
    gp_context_set_error_func(context,contextErrorFunction,this);
    gp_context_set_status_func(context,contextStatusFunction,this);
    emptyContext = gp_context_new();

    //Initialise publishers
    eventpublisher = this->create_publisher<interfaces::msg::Event>("camera_events", 10);
    //Initialise services
    batteryservice = this->create_service<interfaces::srv::IntStatus>(
      "battery_status", std::bind(&DataCamera::battery_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    //Start camera thread
    cameraThread = std::thread(&DataCamera::cameraThreadFunction,this);

    //Announce node start
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera Node starting";
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(),"Node started");
}

DataCamera::~DataCamera()
{
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera Node shutting down";
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(),"Shutdown request received");
    shutdownRequest = true;
    cameraThread.join();
}

bool DataCamera::get_setting_value(char* key, char** value)
{
      bool returnVal = false;
      int ret = 0;
      CameraWidget *widget;
      char* val;

      ret = gp_camera_get_single_config(cameraHandle, key, &widget, context); //Fetch the configuration widget corresponding to that setting
      if (ret == GP_OK)
      {
        ret = gp_widget_get_value(widget, &val);
      }
      if (ret == GP_OK)
      {
        *value = strdup (val ); //TODO: Copied this from previous iteration which contained a note stating that it is unclear what this is or why it is needed. Still don't know so need to find out
      }
      if (ret == GP_OK)
      {
        returnVal = true;
      }
      else if (ret == GP_ERROR_IO_USB_FIND || ret == GP_ERROR_IO_USB_CLAIM ) //TODO: See if this can somehow be incorporated into a context or a callback within libgphoto2. Otherwise this statement will need to go everywhere
      {
        disconnectCamera();
      }

      return returnVal;
}

void DataCamera::contextErrorFunction (GPContext *context, const char *str, void *data)
{
      //data is used to pass in the DataCamera object that called it
      //This is neccessary because libgphoto2 requires this function to be static
      //TODO: might need some way to ensure only a pointer to the class object is passed
      //TODO: I think it should be possible to check if the camera is disconnected here and call the disconnect function
      DataCamera* object = static_cast<DataCamera*> (data);
      RCLCPP_ERROR_STREAM(object->get_logger(), str);
}

void DataCamera::contextStatusFunction (GPContext *context, const char *str, void *data)
{
      //data is used to pass in the DataCamera object that called it
      //This is neccessary because libgphoto2 requires this function to be static
      //TODO: might need some way to ensure only a pointer to the class object is passed
      DataCamera* object = static_cast<DataCamera*> (data);
      RCLCPP_ERROR_STREAM(object->get_logger(), str);
}

void DataCamera::cameraThreadFunction()
{
      //Variable to hold current event
      EventRequest* currentEvent;


      while(shutdownRequest == false)
      {
        if( isCameraConnected == false)
        {
          connectToCamera();
          if (isCameraConnected == false)
          {
            //If camera is not connected, we need to add a delay here to avoid a hot
            //loop in the situation where no camera is connected
            usleep(200); //TODO: Decide whether this should be achieved with a ROS timer
          }
        }
        else if( eventQueue.size() == 0 )
        {
          //Run keep alive command to check camera is still connected
          checkCameraConnection();
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
      RCLCPP_INFO_STREAM(this->get_logger(),"Camera thread closing with " << eventQueue.size() << " events remaining in queue");
}

void DataCamera::connectToCamera()
{
  gp_camera_new(&cameraHandle);
  int ret = gp_camera_init(cameraHandle,emptyContext);
  bool retval = false;
  char *make;
  char *model;
  if (ret == GP_OK)
  {
    //Fetch make and model
    retval = get_setting_value("manufacturer",&make);
    retval += get_setting_value("cameramodel",&model); 
  }
  if(retval == true)
  {
    isCameraConnected = true;
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera connected: " + std::string(make) + " " + std::string(model);
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(),"Camera connected: " + std::string(make) + " " + std::string(model));
  }
  else if(ret == GP_OK)
  {
    isCameraConnected = true;
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera connected but could not fetch make and model";
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(),"Camera connected but could not fetch make and model");
  }
  else
  {
    disconnectCamera();
  }
  return;
}

void DataCamera::disconnectCamera()
{
  if (isCameraConnected == true)
  {
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera disconnected";
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(),"Camera disconnected");
  }
  isCameraConnected = false;
  gp_camera_unref(cameraHandle);
}

void DataCamera::checkCameraConnection()
{
  //Attempt to fetch camera summary
  CameraText cameraSummary;
  int ret = gp_camera_get_summary(cameraHandle,&cameraSummary,context);
  if(ret == GP_ERROR_IO_USB_FIND || ret == GP_ERROR_IO_USB_CLAIM)
  {
    disconnectCamera();
  }
  usleep(500000); //TODO: Determine whether this would be better as a ROS timer or not
  
}

void DataCamera::battery_callback(const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
      std::shared_ptr<interfaces::srv::IntStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(),"Received request for battery status");
  
  //Create event
  batteryRequest event(1,std::string("battery request"),this);
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->value);
  }
  else
  {
    response->value = 0;
    response->status = false;
    response->description = "Need to implement error string here";
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }
  
}