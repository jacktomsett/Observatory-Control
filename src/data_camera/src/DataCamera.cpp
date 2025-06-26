#include "DataCamera.h"
#include "EventsClasses.h"
#include "gphoto2-port-result.h"

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
    eventpublisher = this->create_publisher<interfaces::msg::Event>("camera_events", 10); //TODO: Not currently doing anything with this. At a minimum I would suggets that every service callback reports to this
    //Initialise services
    batteryservice = this->create_service<interfaces::srv::IntStatus>(
      "battery_status", std::bind(&DataCamera::battery_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    getisoservice = this->create_service<interfaces::srv::IntStatus>(
      "get_iso", std::bind(&DataCamera::getiso_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    setisoservice = this->create_service<interfaces::srv::IntRequest>(
      "set_iso", std::bind(&DataCamera::setiso_callback, this, std::placeholders::_1, std::placeholders::_2)
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

bool DataCamera::set_menu_setting_value(char* key, const char* demand, std::string *errorstring)
{
  int ret = 0;
  int Nchoices = 0;
  CameraWidget *widget=NULL;
  CameraWidget *child = NULL;
  std::vector<std::string> allowedValues;
  *errorstring = "";

  //Fetch configuration widget
  ret = gp_camera_get_config(cameraHandle,&widget,context);
  if(ret != GP_OK)
  {
              *errorstring = "Failed to get configuration choice: " + std::string(gp_port_result_as_string(ret));
  }
  else{
    //TODO: The following two function calls are taken from the libgphoto2 github samples. It seems like only one should be necessary, and worth experimenting with.
    ret = gp_widget_get_child_by_name(widget,key,&child);
    if (ret < GP_OK)
    {
      ret = gp_widget_get_child_by_label(widget, key, &child);
    }
  }
  //Build list of allowed values
  if(ret == GP_OK)
  {
    Nchoices = gp_widget_count_choices(child);
    const char * choice;
    for (int i = 0; i < Nchoices; i++)
    {
      ret = gp_widget_get_choice(child,i,&choice);
      if (ret == GP_OK)
      {
        allowedValues.push_back(choice);
      }
      else
      {
        *errorstring = "Failed to get configuration widget: " + std::string(gp_port_result_as_string(ret));
        break;
      }
    }
  }
  //If demand is not present in allowed values, build an error string containing allowed values
  if(!(std::find(allowedValues.begin(),allowedValues.end(),std::string(demand)) != allowedValues.end()))
  {
    *errorstring = "Allowed values are: [";
    for (int i = 0; i < allowedValues.size(); i++)
    {
      if (i != (allowedValues.size()-1))
      {
        *errorstring = *errorstring + allowedValues[i] + ",";
      }
      else
      {
        *errorstring = *errorstring + allowedValues[i] + "]";
      }
    }
  }
  else if(ret == GP_OK)
  {
    ret = gp_widget_set_value(child, demand);
    if(ret != GP_OK)
    {
      *errorstring = "Failed to set value to widget: " + std::string(gp_port_result_as_string(ret));
    }
  }
  if(ret == GP_OK)
  {
    gp_camera_set_config(cameraHandle,widget,context);
    if(ret != GP_OK)
    {
      *errorstring = "Failed to applu new configuration widget to camera: " + std::string(gp_port_result_as_string(ret));
    }
  }

  return (ret == GP_OK);
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
            //FIXME: If a service callback runs while the camera is not connected it will hang indefinitely. There needs to be a service timeout or a check if the camera is connected (or both)
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
          currentEvent->complete = true; //TODO For some reason it is bad practice to directly modify class fields from outside of the class. It is supposed to be done via getter and settor functions. Also maybe this is better controlled by the execute function itself, maybe not (At first I thought not because I dont want the callback function doing anything while the execute function is still running). Either way I havent put any thought into it

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
  //Initialise response status
  response->status = false;

  //Create event
  batteryRequest event(1,std::string("battery request"),response,this);
  //Insert event request into queue
  //TODO: Will probably factor this out into its own function that can be shared amongst callbacks
  queueLock.lock();
  eventQueue.push_back(&event);
  queueLock.unlock();
  while (event.complete == false){}
  if (response->status == true)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->value);
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }
  
}

void DataCamera::getiso_callback(const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
      std::shared_ptr<interfaces::srv::IntStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(),"Received request for iso setting");
  response->status = false;

  //Create event
  getIsoRequest event(1,std::string("iso setting request"), response ,this);
  //Insert event request into queue
  //TODO: Will probably factor this out into its own function that can be shared amongst callbacks. Actually, make it a class member that also sorts event queue via priority
  queueLock.lock();
  eventQueue.push_back(&event);
  queueLock.unlock();
  while (event.complete == false){}
  if (response->status == true)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->value);
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }
  
}

void DataCamera::setiso_callback(const std::shared_ptr<interfaces::srv::IntRequest::Request> request,
      std::shared_ptr<interfaces::srv::IntRequest::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(),"Received demand for iso setting: " << request->demand);
  response->status = false;
    
  //Create event
  setIsoRequest event(1,std::string("iso setting request"),request, response,this);
  //Insert event request into queue
  //TODO: Will probably factor this out into its own function that can be shared amongst callbacks. Actually, make it a class member that also sorts event queue via priority
  queueLock.lock();
  eventQueue.push_back(&event);
  queueLock.unlock();
  while (event.complete == false){}
  if (response->status == true)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->status );
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }
  
}