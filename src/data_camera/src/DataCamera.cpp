#include "DataCamera.h"
#include "EventsClasses.h"
#include "gphoto2-port-result.h"

DataCamera::DataCamera()
: Node("data_camera")
{
    //Initialise variables
  shutdownRequest = false;
  isCameraConnected = false;
  currentSequenceId = "";
  currentSequenceSuccesses = 0;
  currentSequenceFails = 0;

    //Initialise libgphoto contexts
  context = gp_context_new();
  gp_context_set_error_func(context, contextErrorFunction, this);
  gp_context_set_status_func(context, contextStatusFunction, this);
  emptyContext = gp_context_new();

    //Initialise publishers
  eventpublisher = this->create_publisher<interfaces::msg::Event>("camera_events", 10);   //TODO: Not currently doing anything with this. At a minimum I would suggets that every service callback reports to this
    //Initialise services
  batteryservice = this->create_service<interfaces::srv::IntStatus>(
      "battery_status",
    std::bind(&DataCamera::battery_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  getisoservice = this->create_service<interfaces::srv::IntStatus>(
      "get_iso",
    std::bind(&DataCamera::getiso_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  setisoservice = this->create_service<interfaces::srv::IntRequest>(
      "set_iso",
    std::bind(&DataCamera::setiso_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  getqualservice = this->create_service<interfaces::srv::StringStatus>(
      "get_img_quality",
    std::bind(&DataCamera::getqual_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  setqualservice = this->create_service<interfaces::srv::StringRequest>(
      "set_img_quality",
    std::bind(&DataCamera::setqual_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  getfnumberservice = this->create_service<interfaces::srv::StringStatus>(
      "get_f_number",
    std::bind(&DataCamera::getfnumber_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  setfnumberservice = this->create_service<interfaces::srv::StringRequest>(
      "set_f_number",
    std::bind(&DataCamera::setfnumber_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
    //Initialise actions
  requestSequenceAction = rclcpp_action::create_server<interfaces::action::Sequence>(
    this, "sequence",
    std::bind(&DataCamera::sequenceGoal,this, std::placeholders::_1,std::placeholders::_2),
    std::bind(&DataCamera::sequenceCancel,this,std::placeholders::_1),
    std::bind(&DataCamera::sequence_accepted,this,std::placeholders::_1)
  );

        //Pointer to hold current event
  std::shared_ptr<EventRequest> currentEvent(nullptr);
    //Start camera thread
  cameraThread = std::thread(&DataCamera::cameraThreadFunction, this);

    //Announce node start
  auto eventmessage = interfaces::msg::Event();
  eventmessage.event = "Camera Node starting";
  eventpublisher->publish(eventmessage);
  RCLCPP_INFO_STREAM(this->get_logger(), "Node started");
}

DataCamera::~DataCamera()
{
  auto eventmessage = interfaces::msg::Event();
  eventmessage.event = "Camera Node shutting down";
  eventpublisher->publish(eventmessage);
  RCLCPP_INFO_STREAM(this->get_logger(), "Shutdown request received");
  shutdownRequest = true;
  cameraThread.join();
}

bool DataCamera::get_setting_value(char * key, char ** value, std::string *err)
{//TODO: Have not actually checked the error reporting yet because there is no easy way to make a simple request fail currently. Once the focal length service is added it can be tested by running it with the lens detached.
  bool returnVal = false;
  int ret = 0;
  CameraWidget *widget;
  char * val;

  ret = gp_camera_get_single_config(cameraHandle, key, &widget, context);     //Fetch the configuration widget corresponding to that setting
  if (ret == GP_OK) {
    ret = gp_widget_get_value(widget, &val);
  }
  if (ret == GP_OK) {
    *value = strdup (val);      //TODO: Copied this from previous iteration which contained a note stating that it is unclear what this is or why it is needed. Still don't know so need to find out
    returnVal = true;
  } else if (ret == GP_ERROR_IO_USB_FIND || ret == GP_ERROR_IO_USB_CLAIM) {  //TODO: See if this can somehow be incorporated into a context or a callback within libgphoto2. Otherwise this statement will need to go everywhere
    *err = "Camera not found";
    disconnectCamera();
  } else {
    *err = errorstring;
  }

  return returnVal;
}

bool DataCamera::set_menu_setting_value(char * key, const char * demand, std::string *err)
{
  int ret = 0;
  bool invalidDemand = false;
  int Nchoices = 0;
  CameraWidget *widget = NULL;
  CameraWidget *child = NULL;
  std::vector<std::string> allowedValues;
  *err = "";
  char * value;

  //Fetch configuration widget
  ret = gp_camera_get_config(cameraHandle, &widget, context);
  if(ret != GP_OK) {
    *err = "Failed to get configuration choice: " +
      std::string(gp_port_result_as_string(ret)) + " " + errorstring;
  } else {
    //TODO: The following two function calls are taken from the libgphoto2 github samples. It seems like only one should be necessary, and worth experimenting with.
    ret = gp_widget_get_child_by_name(widget, key, &child);
    if (ret < GP_OK) {
      ret = gp_widget_get_child_by_label(widget, key, &child);
    }
  }
  //Build list of allowed values
  if(ret == GP_OK) {
    Nchoices = gp_widget_count_choices(child);
    const char * choice;
    for (int i = 0; i < Nchoices; i++) {
      ret = gp_widget_get_choice(child, i, &choice);
      if (ret == GP_OK) {
        allowedValues.push_back(choice);
      } else {
        *err = "Failed to get configuration widget: " +
          std::string(gp_port_result_as_string(ret)) + " " + errorstring;
        break;
      }
    }
  }
  //If demand is not present in allowed values, build an error string containing allowed values
  if(!(std::find(allowedValues.begin(), allowedValues.end(),
    std::string(demand)) != allowedValues.end()))
  {
    invalidDemand = true;
    *err = "Allowed values are: [";
    for (int i = 0; i < allowedValues.size(); i++) {
      if (i != (allowedValues.size() - 1)) {
        *err = *err + allowedValues[i] + ",";
      } else {
        *err = *err + allowedValues[i] + "]";
      }
    }
  } else if(ret == GP_OK) {
    ret = gp_widget_set_value(child, demand);
    if(ret != GP_OK) {
      *err = "Failed to set value to widget: " + std::string(gp_port_result_as_string(ret)) + " " +
        errorstring;
    }
  }
  if((ret == GP_OK) && (invalidDemand == false)) {
    ret = gp_camera_set_config(cameraHandle, widget, context);
    if(ret != GP_OK) {
      *err = "Failed to apply new configuration widget to camera: " +
        std::string(gp_port_result_as_string(ret)) + " " + errorstring;
    }
  }
  //Check setting reported by camera matches new value
  if((ret == GP_OK) && (invalidDemand == false)) {
    if (get_setting_value(key, &value, err) == false) {
      *err = "Failed to check updated setting value from camera:" + errorstring;
    }
  }
  if( (ret == GP_OK) && (invalidDemand == false) && (strcmp(demand,
    const_cast<char *>(value)) != 0) )
  {
    *err = "Failed to update setting on the camera: " + errorstring;
  }

  return  (ret == GP_OK) && (invalidDemand == false) && (strcmp(demand,
    const_cast<char *>(value)) == 0);
}

void DataCamera::contextErrorFunction(GPContext *context, const char *str, void *data)
{
      //data is used to pass in the DataCamera object that called it
      //This is neccessary because libgphoto2 requires this function to be static
      //TODO: might need some way to ensure only a pointer to the class object is passed
      //TODO: I think it should be possible to check if the camera is disconnected here and call the disconnect function
  DataCamera * object = static_cast<DataCamera *>(data);
  RCLCPP_ERROR_STREAM(object->get_logger(), str);
  object->errorstring = std::string(str);
}

void DataCamera::contextStatusFunction(GPContext *context, const char *str, void *data)
{
      //data is used to pass in the DataCamera object that called it
      //This is neccessary because libgphoto2 requires this function to be static
      //TODO: might need some way to ensure only a pointer to the class object is passed
  DataCamera * object = static_cast<DataCamera *>(data);
  RCLCPP_INFO_STREAM(object->get_logger(), str);
}

void DataCamera::cameraThreadFunction()
{

  while(shutdownRequest == false) {
    if(isCameraConnected == false) {
      connectToCamera();
      if (isCameraConnected == false) {
            //If camera is not connected, we need to add a delay here to avoid a hot
            //loop in the situation where no camera is connected
        usleep(200);     //TODO: Decide whether this should be achieved with a ROS timer
      }
    }
    else if(eventQueue.size() == 0)
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
      //Signal to any waiting processes that the event has finished executing
      currentEvent->complete = true;     //TODO For some reason it is bad practice to directly modify class fields from outside of the class. It is supposed to be done via getter and settor functions. Also maybe this is better controlled by the execute function itself, maybe not (At first I thought not because I dont want the callback function doing anything while the execute function is still running). Either way I havent put any thought into it
      
      //Delete the pointer to current event. It is wrapped in a mutex because certain callbacks (ones that dont create the event they are working with)
      //need to create their own copy of the event pointer otherwise it might be deleted here before that callback can check the value of 'complete'
      currentEventLock.lock();
      currentEvent = nullptr;
      currentEventLock.unlock();

    }
  }
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Camera thread closing with " << eventQueue.size() << " events remaining in queue");
}

void DataCamera::connectToCamera()
{
  gp_camera_new(&cameraHandle);
  int ret = gp_camera_init(cameraHandle, emptyContext);
  bool retval = false;
  char *make;
  char *model;
  std::string err;
  if (ret == GP_OK) {
    //Fetch make and model
    retval = get_setting_value("manufacturer", &make, &err);
    retval += get_setting_value("cameramodel", &model, &err);
  }
  if(retval == true) {
    isCameraConnected = true;
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera connected: " + std::string(make) + " " + std::string(model);
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Camera connected: " + std::string(make) + " " + std::string(model));
  } else if(ret == GP_OK) {
    isCameraConnected = true;
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera connected but could not fetch make and model";
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera connected but could not fetch make and model");
  } else {
    disconnectCamera();
  }
  return;
}

void DataCamera::disconnectCamera()
{
  if (isCameraConnected == true) {
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera disconnected";
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera disconnected");
  }
  isCameraConnected = false;
  gp_camera_unref(cameraHandle);
}

void DataCamera::checkCameraConnection()
{
  //Attempt to fetch camera summary
  CameraText cameraSummary;
  int ret = gp_camera_get_summary(cameraHandle, &cameraSummary, context);
  if(ret == GP_ERROR_IO_USB_FIND || ret == GP_ERROR_IO_USB_CLAIM) {
    disconnectCamera();
  }
  usleep(500000); //TODO: Determine whether this would be better as a ROS timer or not

}

void DataCamera::insertEvent(std::shared_ptr<EventRequest> event)
{
  //TODO: Event queue is not sorting by priority properly anymore
  queueLock.lock();
  eventQueue.push_back(event);
  sort(eventQueue.begin(), eventQueue.end(),[](auto ptr1,auto ptr2){return *ptr1 < *ptr2;});
  queueLock.unlock();
  return;
}

//TODO: Any services that change a setting the camera will need to check if a sequence is currently active and if so these should be rejected. This should be done after the sequence accept logic is built out
void DataCamera::battery_callback(
  const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
  std::shared_ptr<interfaces::srv::IntStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request for battery status");
  //Initialise response status
  response->status = false;
  response->value = 0;
  response->description = "";

  if(isCameraConnected == true) {
    //Create event
    auto eventptr = std::make_shared<batteryRequest>(1, response, this);
    //Insert event request into queue
    insertEvent(eventptr);

    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  } else {
    response->description = "Camera disconnected";
  }
  if (response->status == true) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->value);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }

}

void DataCamera::getiso_callback(
  const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
  std::shared_ptr<interfaces::srv::IntStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request for iso setting");
  response->status = false;
  response->value = 0;
  response->description = "";
  if(isCameraConnected == true) {
    //Create event
    auto eventptr = std::make_shared<getIsoRequest>(1, response, this);
    //Insert event request into queue
    insertEvent(eventptr);

    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  } else {
    response->description = "Camera disconnected";
  }
  if (response->status == true) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->value);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }

}

void DataCamera::setiso_callback(
  const std::shared_ptr<interfaces::srv::IntRequest::Request> request,
  std::shared_ptr<interfaces::srv::IntRequest::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received demand for iso setting: " << request->demand);
  response->status = false;
  response->description = "";
  if(isCameraConnected == true) {
    //Create event
    auto eventptr = std::make_shared<setIsoRequest>(1, request, response, this);
    //Insert event request into queue
    insertEvent(eventptr);

    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  } else {
    response->description = "Camera disconnected";
  }
  if (response->status == true) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->status);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }

}


void DataCamera::getqual_callback(
  const std::shared_ptr<interfaces::srv::StringStatus::Request> request,
  std::shared_ptr<interfaces::srv::StringStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request for image quality setting");
  response->status = false;
  response->value = "";
  response->description = "";
  if(isCameraConnected == true) {
    //Create event
    auto eventptr = std::make_shared<getImgQualityRequest>(1, response, this);
    //Insert event request into queue
    insertEvent(eventptr);

    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  } else {
    response->description = "Camera disconnected";
  }
  if (response->status == true) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->value);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }
}

void DataCamera::setqual_callback(
  const std::shared_ptr<interfaces::srv::StringRequest::Request> request,
  std::shared_ptr<interfaces::srv::StringRequest::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Received demand for image quality setting: " << request->demand);
  response->status = false;
  response->description = "";
  if(isCameraConnected == true) {
    //Create event
    auto eventptr = std::make_shared<setImgQualityRequest>(1, request, response, this);
    //Insert event request into queue
    insertEvent(eventptr);

    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  } else {
    response->description = "Camera disconnected";
  }
  if (response->status == true) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->status);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }
}

void DataCamera::getfnumber_callback(
  const std::shared_ptr<interfaces::srv::StringStatus::Request> request,
  std::shared_ptr<interfaces::srv::StringStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request for f-number setting");
  response->status = false;
  response->value = 0.0;
  response->description = "";
  if(isCameraConnected == true) {
    //Create event
    auto eventptr = std::make_shared<getFNumberRequest>(1, response, this);
    //Insert event request into queue
    insertEvent(eventptr);

    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  } else {
    response->description = "Camera disconnected";
  }
  if (response->status == true) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->value);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }
}

void DataCamera::setfnumber_callback(
  const std::shared_ptr<interfaces::srv::StringRequest::Request> request,
  std::shared_ptr<interfaces::srv::StringRequest::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Received demand for f-number setting: " << request->demand);
  response->status = false;
  response->description = "";
  if(isCameraConnected == true) {
    //Create event
    auto eventptr = std::make_shared<setFNumberRequest>(1, request, response, this);
    //Insert event request into queue
    insertEvent(eventptr);

    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  } else {
    response->description = "Camera disconnected";
  }
  if (response->status == true) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with " << response->status);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Responding with fail status");
  }
}

rclcpp_action::GoalResponse DataCamera::sequenceGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const interfaces::action::Sequence::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Received request for photo sequence containing " << goal->length << " photos");
  //TODO: Placeholder function. This just accepts all goals. Unlike the previous iteration I do think this wouldnt actually break anything as new sequences would just get queued for after existing sequences. Regardless ultimatly this should be overhauled and a state flag added to DataCamera that tracks what sequences have currently been accepted and rejects new ones unless they are sent with high priority or have explicitly said they are happy to wait until the current one finishes
  (void) uuid;
  RCLCPP_INFO_STREAM(this->get_logger(), "Sequence request accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DataCamera::sequenceCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goalHandle)
{
  //FIXME: This does remove all the events from the queue, but if the currently being acted on event execute function is part of this sequence (likely) then it will continue. This eventually results in it trying to publish feedback to a goal that does not exist
  //Also, I don't know if the issue is with this code, but the ROS2 CLI action program doesn't quit after the sequence has been cancelled. I don't know if it is waiting for the action server to send some sort of notification
  //FIXME: After fixing the service issues, The goal no longer cancels properly.
  RCLCPP_INFO_STREAM(this->get_logger(), "Cancelling photo sequence");
  std::string goalID = rclcpp_action::to_string(goalHandle->get_goal_id());

  //Remove all events associated with this goal from the eventQueue
  queueLock.lock();
  for(int i = eventQueue.size() - 1; i != 0; i--) { //TODO: Might be more efficient to run through the queue backwards
    if (goalID == eventQueue[i]->ID) 
    {
      eventQueue.erase(eventQueue.begin() + i);
    }
  }
  queueLock.unlock();
  //Check if an event is currently executing (ie currentEvent != nullptr) and if it is part of this goal, if so then we should wait for that to finish
  currentEventLock.lock();
  if(currentEvent && currentEvent->ID == goalID)
  {
    //Increase the reference count of pointer so that it wont be destroyed before the comparison can be done
    std::shared_ptr<EventRequest> eventptr = currentEvent;
    currentEventLock.unlock();
    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  }
  currentEventLock.unlock();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DataCamera::sequence_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goalHandle)
{
  //This callback needs to finish quickly so it doesn't freeze up the system, so instead of populating the event queue with all of
  //the photo requests here, we will add in a single event that in turn will generste the rest of the events
  auto eventptr = std::make_shared<generateSequence>(1, goalHandle, this);
  insertEvent(eventptr);
}
