#include "DataCamera.h"
#include "EventsClasses.h"
#include "gphotoWrapper.h"
#include "gphoto2-port-result.h"

DataCamera::DataCamera()
: Node("data_camera"),
	camera(this)
{
    //Initialise variables
  shutdownRequest = false;
  currentSequenceId = "";
  currentSequenceSuccesses = 0;
  currentSequenceFails = 0;


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
  getexposureservice = this->create_service<interfaces::srv::StringStatus>(
      "get_exposure",
    std::bind(&DataCamera::getexposure_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  setexposureservice = this->create_service<interfaces::srv::StringRequest>(
      "set_exposure",
    std::bind(&DataCamera::setexposure_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  getfocusmodeservice = this->create_service<interfaces::srv::StringStatus>(
      "get_focus_mode",
    std::bind(&DataCamera::getfocusmode_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  setfocusmodeservice = this->create_service<interfaces::srv::StringRequest>(
      "set_focus_mode",
    std::bind(&DataCamera::setfocusmode_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
    //Initialise actions
  requestSequenceAction = rclcpp_action::create_server<interfaces::action::Sequence>(
    this, "sequence",
    std::bind(&DataCamera::sequenceGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DataCamera::sequenceCancel, this, std::placeholders::_1),
    std::bind(&DataCamera::sequence_accepted, this, std::placeholders::_1)
  );

        //Pointer to hold current event
  std::shared_ptr<EventRequest> currentEvent(nullptr);
    //Start camera thread
  cameraThread = std::thread(&DataCamera::cameraThreadFunction, this);

    //Announce node start
  RCLCPP_INFO_STREAM(this->get_logger(), "Node started");
  auto eventmessage = interfaces::msg::Event();
  eventmessage.event = "Camera Node starting"; //FIXME: For some reason this isn't getting published
  eventpublisher->publish(eventmessage);
}

DataCamera::~DataCamera()
{
  //FIXME: When a sequence goal is active, sending a shutdown request causes a crash rather than a graceful shutdown
  auto eventmessage = interfaces::msg::Event();
  eventmessage.event = "Camera Node shutting down";
  eventpublisher->publish(eventmessage); //FIXME: For some reason this isn't getting published
  RCLCPP_INFO_STREAM(this->get_logger(), "Shutdown request received");
  shutdownRequest = true;
  cameraThread.join();
}

bool DataCamera::get_setting_value(std::string param, std::string* value_ptr, std::string* error_string_ptr)
{
	//TODO: This seems unelegent. Perhaps there is a way to automatically expose the wrappers api to the node class
	*error_string_ptr = "ERROR PARSING NOT REIMPLEMENTED YET";
	int ret = camera.getSettingValue(param,value_ptr);
	if (ret == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DataCamera::set_setting_value(std::string param, std::string value, std::string* error_string_ptr)
{
	*error_string_ptr = "ERROR PARSING NOT REIMPLEMENTED YET";
	int ret = camera.setSettingValue(param,value);
	if (ret == 0)
	{
		return true;
	}
	else
	{
		return false;
	}	
}

bool DataCamera::capture_image()
{
	int ret = camera.captureImage();
	if (ret == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}
void DataCamera::cameraThreadFunction()
{

  while(shutdownRequest == false) {
    if(camera.isCameraPresent() == false) {
      camera.checkConnection();
      if (camera.isCameraPresent() == false) {
            //If camera is not connected, we need to add a delay here to avoid a hot
            //loop in the situation where no camera is connected
        usleep(200);     //TODO: Decide whether this should be achieved with a ROS timer
      }
    } else if(eventQueue.size() == 0) {
          //Run keep alive command to check camera is still connected
      camera.checkConnection();
    } else {
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

/*void DataCamera::connectToCamera()
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
}*/

/*void DataCamera::disconnectCamera()
{
  if (isCameraConnected == true) {
    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Camera disconnected";
    eventpublisher->publish(eventmessage);
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera disconnected");
  }
  isCameraConnected = false;
  gp_camera_unref(cameraHandle);
}*/

/*void DataCamera::checkCameraConnection()
{
  //Attempt to fetch camera summary
  CameraText cameraSummary;
  int ret = gp_camera_get_summary(cameraHandle, &cameraSummary, context);
  if(ret == GP_ERROR_IO_USB_FIND || ret == GP_ERROR_IO_USB_CLAIM) {
    disconnectCamera();
  }
  usleep(500000); //TODO: Determine whether this would be better as a ROS timer or not

}*/

void DataCamera::insertEvent(std::shared_ptr<EventRequest> event)
{
  //TODO: Event queue is not sorting by priority properly anymore
  queueLock.lock();
  eventQueue.push_back(event);
  sort(eventQueue.begin(), eventQueue.end(), [](auto ptr1, auto ptr2){return *ptr1 < *ptr2;});
  queueLock.unlock();
  return;
}

//TODO: Any services that change a setting the camera will need to check if a sequence is currently active and if so these should be rejected. This should be done after the sequence accept logic is built out
//Initially I thought the priority system would handle this but I now realise that wont work because even if the EventRequest is succesfully put to the end of the queue after the sequence, the ROS service will
//still hang until it's completed which is not good. Alternative options would be that the service can return and effectively confirm that its been put in the queue and the client will have to accept that they
//wont have any confirmation that it's been successfully done, or alternatively the service can be rejected entirely and the desired functionality (ie the ability to be able to queue up a parameter change for after
// a currently executing sequence) can be wrapped up into a new action definition called something like sequenceRequestWithParameters where in addition to adding in the events for capturing the photos, the required parameter
// settting events are also added and the sequence will only proceed if they are all successfull (I think I will go with this one)
void DataCamera::battery_callback(
  const std::shared_ptr<interfaces::srv::IntStatus::Request> request,
  std::shared_ptr<interfaces::srv::IntStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request for battery status");
  //Initialise response status
  response->status = false;
  response->value = 0;
  response->description = "";

  if(camera.isCameraPresent() == true) {
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
  if(camera.isCameraPresent() == true) {
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
  if(camera.isCameraPresent() == true) {
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
  if(camera.isCameraPresent() == true) {
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
  if(camera.isCameraPresent() == true) {
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
  response->value = "";
  response->description = "";
  if(camera.isCameraPresent() == true) {
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
  if(camera.isCameraPresent() == true) {
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

void DataCamera::getexposure_callback(
  const std::shared_ptr<interfaces::srv::StringStatus::Request> request,
  std::shared_ptr<interfaces::srv::StringStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request for exposure setting");
  response->status = false;
  response->value = "";
  response->description = "";
  if(camera.isCameraPresent() == true) {
    //Create event
    auto eventptr = std::make_shared<getExposureRequest>(1, response, this);
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

void DataCamera::setexposure_callback(
  const std::shared_ptr<interfaces::srv::StringRequest::Request> request,
  std::shared_ptr<interfaces::srv::StringRequest::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Received demand for exposure setting: " << request->demand);
  response->status = false;
  response->description = "";
  if(camera.isCameraPresent() == true) {
    //Create event
    auto eventptr = std::make_shared<setExposureRequest>(1, request, response, this);
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

void DataCamera::getfocusmode_callback(
  const std::shared_ptr<interfaces::srv::StringStatus::Request> request,
  std::shared_ptr<interfaces::srv::StringStatus::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request for focus mode setting");
  response->status = false;
  response->value = "";
  response->description = "";
  if(camera.isCameraPresent() == true) {
    //Create event
    auto eventptr = std::make_shared<getFocusModeRequest>(1, response, this);
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

void DataCamera::setfocusmode_callback(
  const std::shared_ptr<interfaces::srv::StringRequest::Request> request,
  std::shared_ptr<interfaces::srv::StringRequest::Response> response)
{
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Received demand for focus mode setting: " << request->demand);
  response->status = false;
  response->description = "";
  if(camera.isCameraPresent() == true) {
    //Create event
    auto eventptr = std::make_shared<setFocusModeRequest>(1, request, response, this);
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
  //TODO: Add a field to the action definition to include a parameter specify what to do in the case that there is an existing action (eg queue it for after the existing action or cancel the current action first) ((actually thats not a bad set of options for all camera setting changes))
  rclcpp_action::GoalResponse response;
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Received request for photo sequence containing " << goal->length << " photos");
  if (currentSequenceId == "") { //TODO: Should also reject sequence if no camera is connected
    (void) uuid; //TODO: Find out why this is cast to void in the example
    RCLCPP_INFO_STREAM(this->get_logger(), "Sequence request accepted");
    response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Sequence of " + std::to_string(goal->length) + " accepted";
    eventpublisher->publish(eventmessage);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Sequence request denied due to existing sequence being processed");
    response = rclcpp_action::GoalResponse::REJECT;

    auto eventmessage = interfaces::msg::Event();
    eventmessage.event = "Sequence of " + std::to_string(goal->length) +
      " rejected: Sequence already in progress";
    eventpublisher->publish(eventmessage);
  }

  return response;
}

rclcpp_action::CancelResponse DataCamera::sequenceCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goalHandle)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Cancelling photo sequence");
  std::string goalID = rclcpp_action::to_string(goalHandle->get_goal_id());

  //Remove all events associated with this goal from the eventQueue
  queueLock.lock();
  for(int i = eventQueue.size() - 1; i != 0; i--) {
    if (goalID == eventQueue[i]->ID) {
      eventQueue.erase(eventQueue.begin() + i);
    }
  }
  queueLock.unlock();
  //Check if an event is currently executing (ie currentEvent != nullptr) and if it is part of this goal, if so then we should wait for that to finish
  currentEventLock.lock();
  if(currentEvent && currentEvent->ID == goalID) {
    //Increase the reference count of pointer so that it wont be destroyed before the comparison can be done
    std::shared_ptr<EventRequest> eventptr = currentEvent;
    currentEventLock.unlock();
    while (eventptr->complete == false) {
      usleep(1); //TODO: Added in an attempt to stabilise thread sync. Just an experiment, I know ultimately more mutexes are needed.
    }
  }
  currentEventLock.unlock();

  //Finally, clear the sequence information fields
  currentSequenceId = "";
  currentSequencePhotoNumber = 0;
  currentSequenceSuccesses = 0;
  currentSequenceFails = 0;

  auto eventmessage = interfaces::msg::Event();
  eventmessage.event = "Sequence aborted";
  eventpublisher->publish(eventmessage);

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
