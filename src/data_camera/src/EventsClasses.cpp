#include "EventsClasses.h"
#include "DataCamera.h"

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