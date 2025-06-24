#include "EventsClasses.h"
#include "DataCamera.h"

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