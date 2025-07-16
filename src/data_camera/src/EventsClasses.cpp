#include "EventsClasses.h"
#include "DataCamera.h"
#include <chrono>
#include <format>

EventRequest::EventRequest()
:priority(3),
  timestamp(std::format("{:%FT%TZ}", std::chrono::system_clock::now())),
  complete(false),
  cameranode(nullptr)
{}

EventRequest::EventRequest(int p, DataCamera * node)
:priority(p),
  timestamp(std::format("{:%FT%TZ}", std::chrono::system_clock::now())),
  complete(false),
  cameranode(node)
{

}

EventRequest::~EventRequest() {}
bool EventRequest::operator<(const EventRequest & b)
{
  bool result;
  if (b.priority == this->priority) {
    result = (this->timestamp > b.timestamp);
  } else {
    result = (this->priority) > b.priority;
  }
  return result;
}

batteryRequest::batteryRequest(
  int p, std::shared_ptr<interfaces::srv::IntStatus::Response> res,
  DataCamera * node)
{
  priority = p;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  complete = false;
  response = res;
  cameranode = node;
}

batteryRequest::~batteryRequest() {}

void batteryRequest::execute()
{
  char * batteryValue;
  std::string error = "";
  bool retval = cameranode->get_setting_value("batterylevel", &batteryValue, &error);
  if (retval == true) {
    std::string battValue = std::string(batteryValue); \
    battValue.pop_back();   //Remove percent sign
    response->value = std::stoi(battValue);
    response->description = "";
    response->status = true;
  } else {
    response->value = 0;
    response->status = false;
    response->description = error;
  }
  return;
}

getIsoRequest::getIsoRequest(
  int p, std::shared_ptr<interfaces::srv::IntStatus::Response> res, DataCamera * node)
{
  priority = p;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  complete = false;
  cameranode = node;
  response = res;
}

getIsoRequest::~getIsoRequest() {}

void getIsoRequest::execute()
{
  char * isoSettingValue;
  std::string error = "";
  bool retval = cameranode->get_setting_value("iso", &isoSettingValue, &error);
  if (retval == true) {
    response->value = std::stoi(isoSettingValue);
    response->description = "";
    response->status = true;
  } else {
    response->value = 0;
    response->description = error;
    response->status = false;
  }
  return;
}

setIsoRequest::setIsoRequest(
  int p, std::shared_ptr<interfaces::srv::IntRequest::Request> req,
  std::shared_ptr<interfaces::srv::IntRequest::Response> res, DataCamera * node)
{
  priority = p;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  cameranode = node;
  request = req;
  response = res;
}

setIsoRequest::~setIsoRequest() {}

void setIsoRequest::execute()
{
  std::string error, demandstring;
  demandstring = std::to_string(request->demand);
  const char * dem = (demandstring).c_str();
  bool retVal = cameranode->set_menu_setting_value("iso", dem, &error);

  if( (retVal == true)) {
    response->status = true;
    response->description = "";
  } else {
    response->status = false;
    response->description = error;
  }

}

getImgQualityRequest::getImgQualityRequest(
  int p,
  std::shared_ptr<interfaces::srv::StringStatus::Response> res, DataCamera * node)
{
  priority = p;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  complete = false;
  response = res;
  cameranode = node;
}

getImgQualityRequest::~getImgQualityRequest() {}

void getImgQualityRequest::execute()
{
  char * qualitySettingValue;
  std::string error = "";
  bool retval = cameranode->get_setting_value("imagequality", &qualitySettingValue, &error);
  if (retval == true) {
    response->value = std::string(qualitySettingValue);
    response->description = "";
    response->status = true;
  } else {
    response->value = "ERR";
    response->description = error;
    response->status = false;
  }
  return;
}

setImgQualityRequest::setImgQualityRequest(
  int p,
  std::shared_ptr<interfaces::srv::StringRequest::Request> req,
  std::shared_ptr<interfaces::srv::StringRequest::Response> res, DataCamera * node)
{
  priority = p;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  cameranode = node;
  request = req;
  response = res;
}

setImgQualityRequest::~setImgQualityRequest() {}

void setImgQualityRequest::execute()
{
  std::string error;
  const char * dem = (request->demand).c_str();
  bool retVal = cameranode->set_menu_setting_value("imagequality", dem, &error);

  if( (retVal == true)) {
    response->status = true;
    response->description = "";
  } else {
    response->status = false;
    response->description = error;
  }
}

getFNumberRequest::getFNumberRequest(
  int p,
  std::shared_ptr<interfaces::srv::StringStatus::Response> res, DataCamera * node)
{
  priority = p;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  complete = false;
  response = res;
  cameranode = node;
}

getFNumberRequest::~getFNumberRequest() {}

void getFNumberRequest::execute()
{
  char * fNumberSettingValue;
  std::string error = "";
  bool retval = cameranode->get_setting_value("f-number", &fNumberSettingValue, &error);
  if (retval == true) {
    response->value = std::string(fNumberSettingValue);
    response->description = "";
    response->status = true;
  } else {
    response->value = 0.0;
    response->description = error;
    response->status = false;
  }
  return;
}

setFNumberRequest::setFNumberRequest(
  int p,
  std::shared_ptr<interfaces::srv::StringRequest::Request> req,
  std::shared_ptr<interfaces::srv::StringRequest::Response> res, DataCamera * node)
{
  priority = p;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  cameranode = node;
  request = req;
  response = res;
}

setFNumberRequest::~setFNumberRequest() {}

void setFNumberRequest::execute()
{
  std::string error;
  const char * dem = (request->demand).c_str();
  bool retVal = cameranode->set_menu_setting_value("f-number", dem, &error);

  if( (retVal == true)) {
    response->status = true;
    response->description = "";
  } else {
    response->status = false;
    response->description = error;
  }
}

sequencePhotoRequest::sequencePhotoRequest(
  int p, int n, std::string goalID,
  std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> gh,
  DataCamera * node)
{
  priority = p;
  photoNumber = n;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  ID = goalID;
  goalHandle = gh;
  cameranode = node;
}

sequencePhotoRequest::~sequencePhotoRequest() {
}

void sequencePhotoRequest::execute()
{
  //Fetch goal and feedback. //TODO: This is copied from previous iteration (and ultimately before that the ROS2 tutorial), I want to understand exactly what it is doing a bit better
  const auto goal = goalHandle->get_goal();//Also while doing addressing the above comment, decide whether it would be better to generate these pointers in the generateSequence class and pass them to each of the photoRequest class objects
  auto feedback = std::make_shared<interfaces::action::Sequence::Feedback>();

  bool ret = cameranode->capture_image();
  //Update counts
  cameranode->currentSequencePhotoNumber++;
  if (ret == true) {
    cameranode->currentSequenceSuccesses++;
  } else {
    cameranode->currentSequenceFails++;
  }
  //TODO: Don't know if here is where it would go, but it would be nice if we could set a threshold (eg.5) where if we get 5 failed captures in a row the sequence will abort (cancel)
  feedback->current = cameranode->currentSequencePhotoNumber;
  feedback->successes = cameranode->currentSequenceSuccesses;
  feedback->fails = cameranode->currentSequenceFails;
  goalHandle->publish_feedback(feedback);

  //Check if goal is complete
  if(cameranode->currentSequencePhotoNumber == goal->length) { //TODO: Alternatively this could be currentImage == goal-> length. Or both. A mechanism should be put in place in case the images for some reason end up out of order in the event queue
    cameranode->currentSequenceId = "";
    cameranode->currentSequencePhotoNumber = 0;
    cameranode->currentSequenceSuccesses = 0;
    cameranode->currentSequenceFails = 0;    
    
    auto exitStatus = std::make_shared<interfaces::action::Sequence::Result>();
    exitStatus->confirmcomplete = "Sequence complete";
    exitStatus->successes = feedback->successes;
    exitStatus->fails = feedback->fails;
    goalHandle->succeed(exitStatus);
  }
  return;
}

generateSequence::generateSequence(
  int p,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> gh,
  DataCamera * node)
{
  priority = p;
  timestamp = std::format("{:%FT%TZ}", std::chrono::system_clock::now());
  goalHandle = gh;
  ID = rclcpp_action::to_string(goalHandle->get_goal_id());
  cameranode = node;
}

generateSequence::~generateSequence() {}

void generateSequence::execute()
{
  const auto goal = goalHandle->get_goal();
  cameranode->currentSequenceId = ID;
  cameranode->currentSequencePhotoNumber = 0;
  cameranode->currentSequenceSuccesses = 0;
  cameranode->currentSequenceFails = 0;
  for (int i = 1; i <= goal->length; i++) {
    auto eventptr = std::make_shared<sequencePhotoRequest>(2, i, ID, goalHandle, cameranode);
    cameranode->insertEvent(eventptr);
  }
}
