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
//TODO: Need to verify this operator actually works correctly.
bool EventRequest::operator<(const EventRequest & b)
{
  bool result;
  if (b.priority == this->priority) {
    result = (this->timestamp < b.timestamp);
  } else {
    result = (this->priority) < b.priority;
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
