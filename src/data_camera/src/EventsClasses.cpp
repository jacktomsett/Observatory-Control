#include "EventsClasses.h"
#include "DataCamera.h"

EventRequest::EventRequest()
  :priority(3),
  complete(false),
  status(false),
  cameranode(nullptr),
  eventID("ERROR"),
  result("")
{};

EventRequest::EventRequest(int p, std::string n, DataCamera* node)
  :priority(p),
  complete(false),
  status(false),
  cameranode(node),
  eventID(n),
  result("")
{};

EventRequest::~EventRequest(){};

batteryRequest::batteryRequest(int p, std::string n, DataCamera* node)
{
  priority=p;
  complete=false;
  status=false;
  cameranode=node;
  eventID=n;
  result="";
};

batteryRequest::~batteryRequest(){};

void batteryRequest::execute()
{
    char* batteryValue;
    bool retval = cameranode->get_setting_value("batterylevel",&batteryValue);
    if (retval == true)
    {
      result = std::string(batteryValue);
      status = true;
    }
    else
    {
      result = "0";
      status = false;
    }
    return;
}

getIsoRequest::getIsoRequest(int p, std::string n, DataCamera* node)
{
  priority=p;
  complete=false;
  status=false;
  cameranode=node;
  eventID=n;
  result="";
};

getIsoRequest::~getIsoRequest(){};

void getIsoRequest::execute()
{
    char* isoSettingValue;
    bool retval = cameranode->get_setting_value("iso",&isoSettingValue);
    if (retval == true)
    {
      result = std::string(isoSettingValue);
      status = true;
    }
    else
    {
      result = "0";
      status = false;
    }
    return;
}

setIsoRequest::setIsoRequest(int p, std::string n, int d, DataCamera* node)
{
  priority=p;
  complete=false;
  status=false;
  cameranode=node;
  eventID=n;
  result="";
  demand = d;
};

setIsoRequest::~setIsoRequest(){};

void setIsoRequest::execute()
{
  result = "Error, functionality not yet implemented";
  return;
}