#include "EventsClasses.h"
#include "DataCamera.h"

EventRequest::EventRequest()
  :priority(3),
  complete(false),
  cameranode(nullptr),
  eventID("ERROR")
{};

EventRequest::EventRequest(int p, std::string n, DataCamera* node)
  :priority(p),
  complete(false),
  cameranode(node),
  eventID(n)
{};

EventRequest::~EventRequest(){};

batteryRequest::batteryRequest(int p, std::string n, std::shared_ptr<interfaces::srv::IntStatus::Response> res, DataCamera* node)
{
  priority=p;
  complete=false;
  response = res;
  cameranode=node;
  eventID=n;
};

batteryRequest::~batteryRequest(){};

void batteryRequest::execute()
{
    char* batteryValue;
    bool retval = cameranode->get_setting_value("batterylevel",&batteryValue);
    if (retval == true)
    {
      std::string battValue = std::string(batteryValue);\
      battValue.pop_back(); //Remove percent sign
      response->value = std::stoi(battValue);
      response->description = "";
      response->status = true;
    }
    else
    {
      response->value = 0;
      response->status = false;
      response->description = "Error fetching value from camera"; //TODO: Would be nice to have the libgphoto error here. Would need to pass the Event class to the context (or just response pointer perhaps)
    }
    return;
}

getIsoRequest::getIsoRequest(int p, std::string n, std::shared_ptr<interfaces::srv::IntStatus::Response> res, DataCamera* node)
{
  priority=p;
  complete=false;
  cameranode=node;
  eventID=n;
  response = res;
};

getIsoRequest::~getIsoRequest(){};

void getIsoRequest::execute()
{
    char* isoSettingValue;
    bool retval = cameranode->get_setting_value("iso",&isoSettingValue);
    if (retval == true)
    {
      response->value = std::stoi(isoSettingValue);
      response->description = "";
      response->status = true;
    }
    else
    {
      response->value = 0;
      response->description = "Error fetching information from camera"; //TODO: Pull from libgphoto
      response->status = false;
    }
    return;
}

setIsoRequest::setIsoRequest(int p, std::string n, std::shared_ptr<interfaces::srv::IntRequest::Request> req, std::shared_ptr<interfaces::srv::IntRequest::Response> res, DataCamera* node)
{
  priority=p;
  complete=false;
  cameranode=node;
  eventID=n;
  request = req;
  response = res;
};

setIsoRequest::~setIsoRequest(){};

void setIsoRequest::execute()
{
  std::string errorstring, demandstring;
  demandstring = std::to_string(request->demand);
  const char* dem = (demandstring).c_str();
  bool retVal = cameranode->set_menu_setting_value("iso",dem,&errorstring);
  if(retVal == true)
  {
    response->status = true;
    response->description = "";
  }
  else
  {
    response->status = false;
    response->description = errorstring;
  }

  return;
}