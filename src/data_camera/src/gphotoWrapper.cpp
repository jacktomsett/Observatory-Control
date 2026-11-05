#include "gphoto2/gphoto2.h"
#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2-context.h>
#include <gphoto2/gphoto2-port-result.h>
#include <gphoto2/gphoto2-widget.h>
#include <iostream>
#include <string.h>
#include <vector>
#include "gphotoWrapper.h"




gphotoWrapper::gphotoWrapper()
{
	gp_camera_new(&camera);
	context = gp_context_new();
	gp_context_set_error_func(context,ctx_error_func,NULL);
	gp_context_set_status_func(context, ctx_status_func, NULL);
	topLevelWidget = NULL;
	cameraPresentFlag = false;

	checkConnection();
	return;

};

gphotoWrapper::~gphotoWrapper()
{
	if(cameraPresentFlag == true)
	{
		gp_widget_free(topLevelWidget);
	}
	gp_camera_exit(camera, context);
};

bool gphotoWrapper::isCameraPresent()
{
	return cameraPresentFlag;
}

int gphotoWrapper::checkConnection()
{
	int retVal = 1;
	int gphotoRet;
	if(cameraPresentFlag == true)
	{
		//If camera is currently assumed to be connected, then attempt to retrieve a summary
		//TODO: Would like for the class to hold a copy of the camera serial number and warn for any changes to it
		CameraText summaryText;
		gphotoRet = gp_camera_get_summary(camera,&summaryText,context);
		if (gphotoRet == GP_ERROR_IO_USB_FIND || gphotoRet == GP_ERROR_IO_USB_CLAIM)
		{
			std::cerr << "Camera connection lost" << std::endl;
			cameraPresentFlag = false;
			//gp_camera_free(camera);
			retVal = 0;
		}
		else if(gphotoRet < GP_OK)
		{
			std::cerr << "Unexpected error when checking camera connection" << std::endl;
			//TODO: Process error
		}
		else
		{
			retVal = 0;
		}

	}
	else
	{
		//If camera is currently assumed to be absent, attempt to init the class the camera connection
		gphotoRet = gp_camera_init(camera, context);
		if (gphotoRet == GP_OK)
		{
			gphotoRet = gp_camera_get_config(camera,&topLevelWidget,context);
			if (gphotoRet == GP_OK)
			{
				cameraPresentFlag = true;
				retVal = 0;
			}
			else
			{
				std::cerr << "Failed to fetch summary of newly connected camera" << std::endl;
				retVal = 1;
				cameraPresentFlag = false;
				//gp_camera_free(camera);

			}

		}
		
	
		//else if (gphotoRet == GP_ERROR_IO_USB_FIND || gphotoRet == GP_ERROR_IO_USB_CLAIM)
		else if (gphotoRet < 0) //FIXME, currently this is returning a value of -105 which isnt defined in the gphoto library. Investigate
		{
			cameraPresentFlag = false;
			retVal = 0;
		}
		else
		{
			cameraPresentFlag = false;
			retVal = 1;
			std::cerr << "Unexpected error whilst checking camera connection" << std::endl;
			//TODO: Process error
		}
	}
	return retVal;
};


int gphotoWrapper::getSettingValue(std::string parameter, std::string* value_ptr)
{
	if(!cameraPresentFlag)
	{
		*value_ptr = "ERROR: NO CAMERA";
		return 1;
	}
	//Create widget and convert the param to c string
	CameraWidget *widget = NULL;
	CameraWidgetType widgetType;
	const char *key = parameter.c_str();

	int GPret;
	int ret;
	GPret = gp_widget_get_child_by_label(topLevelWidget, key, &widget);
	if (GPret < GP_OK)
	{
		GPret = gp_widget_get_child_by_name(topLevelWidget, key, &widget);
	}
	if (GPret < GP_OK)
	{
		std::cerr << "lookup widget failed" << std::endl;
		return 1;
	}
	//Check widget type
	GPret = gp_widget_get_type(widget,&widgetType);
	if (GPret < GP_OK)
	{
		std::cerr << "widget get type failed" << std::endl;
		return 1;
	}

	//Variable to store result
	char* val;
	char* str;
	switch(widgetType)
	{
		case GP_WIDGET_TEXT:
			ret = getTextConfig(widget,value_ptr);
			break;

		case GP_WIDGET_RANGE:
			ret = getRangeConfig(widget, value_ptr);
			break;

		case GP_WIDGET_TOGGLE:
			//ret = getToggleConfig(widget, value_ptr);
			std::cout << "Widget Type Not Implemented Yet" << std::endl;
			ret = 1;

			break;

		case GP_WIDGET_RADIO:
			ret = getTextConfig(widget,value_ptr);
			break;

		case GP_WIDGET_MENU:
			ret = getTextConfig(widget,value_ptr);
			break;

		case GP_WIDGET_BUTTON:
			//ret = getButtonConfig(widget, &val);
			std::cout << "Widget Type Not Implemented Yet" << std::endl;
			ret = 1;
			break;

		case GP_WIDGET_DATE:
			//ret = getDateConfig(widget, &val);
			std::cout << "Widget Type Not Implemented Yet" << std::endl;
			ret = 1;
			break;

	}
	if (ret != GP_OK)
	{
		//Process Error Here
		return 1;
	}
	return 0;
};

int gphotoWrapper::setSettingValue(std::string parameter, std::string newValue)
{
	if(!cameraPresentFlag)
	{
		std::cerr << "ERROR: setSettingValue called while camera is not connected." << std::endl;
		//TODO: Since this should not be possible, should we throw an exception here?
		return 1;
	}
	//Create widget and convert the param to c string
	CameraWidget *widget = NULL;
	CameraWidgetType widgetType;
	const char *key = parameter.c_str();

	int GPret;
	int retVal;
	GPret = gp_widget_get_child_by_label(topLevelWidget, key, &widget);
	if (GPret < GP_OK)
	{
		GPret = gp_widget_get_child_by_name(topLevelWidget, key, &widget);
	}
	if (GPret < GP_OK)
	{
		std::cerr << "lookup widget failed" << std::endl;
		gp_widget_free(widget);
		return 1;
	}
	//Check widget type
	GPret = gp_widget_get_type(widget,&widgetType);
	if (GPret < GP_OK)
	{
		std::cerr << "widget get type failed" << std::endl;
		return 1;
		gp_widget_free(widget);
	}

	switch(widgetType)
	{
				case GP_WIDGET_TEXT:
					//ret = setTextConfig(widget, newValue);
					std::cout << "Widget Type Not Implemented Yet" << std::endl;
					retVal = 1;

					break;

				case GP_WIDGET_RANGE:
					//ret = setRangeConfig(widget, newValue);
					std::cout << "Widget Type Not Implemented Yet" << std::endl;
					retVal = 1;

					break;
				case GP_WIDGET_TOGGLE:
					//ret = setToggleConfig(widget, newValue);
					std::cout << "Widget Type Not Implemented Yet" << std::endl;
					retVal = 1;

					break;
				case GP_WIDGET_RADIO:
					retVal = setRadioConfig(widget, newValue);
					break;
				case GP_WIDGET_MENU:
					retVal = setRadioConfig(widget, newValue);
					break;
				case GP_WIDGET_BUTTON:
					//ret = setButtonConfig(widget, newValue);
					std::cout << "Widget Type Not Implemented Yet" << std::endl;
					retVal = 1;

					break;
				case GP_WIDGET_DATE:
					//ret = setDateConfig(widget, newValue);
					std::cout << "Widget Type Not Implemented Yet" << std::endl;
					retVal = 1;
					break;

	}
	//gp_widget_free(widget);
	return retVal;

};


int gphotoWrapper::captureImage()
{
				int gphotoRet, retVal;
				CameraFilePath camera_file_path;

				strcpy(camera_file_path.folder, "/");
				strcpy(camera_file_path.name, "foo.jpg");

				gphotoRet = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
				if (gphotoRet < GP_OK)
				{
								std::cerr << "Error capturing photo" << std::endl;
								retVal = 1;
				}
				else
				{
								std::cout << "Image captured" << std::endl;
								retVal = 0;
				}
				return retVal;
}

//=================Private member functions==========================================//
void gphotoWrapper::ctx_error_func(GPContext *context, const char *str, void *data)
{
	std::cerr << "******* Contexterror ******" << std::endl << str << std::endl;
};

void gphotoWrapper::ctx_status_func(GPContext *context, const char *str, void *data)
{
	std::cout << "******* ContextStatus ******" << std::endl << str << std::endl;
};

int gphotoWrapper::getTextConfig(CameraWidget *widget, std::string* fetchedValue_ptr)
{
	char *str;
	int ret = gp_widget_get_value(widget, &str);
	if(ret < GP_OK)
	{
		std::cerr << "Error fetching widget value" << std::endl;
		return 1;
	}
	*fetchedValue_ptr = std::string(str);
	return 0;
}

int gphotoWrapper::getRangeConfig(CameraWidget *widget, std::string* fetchedValue_ptr)
{
				float rangeVal;
				int ret = gp_widget_get_value(widget, &rangeVal);
				if(ret < GP_OK)
				{
								std::cerr << "Error fetching widget value" << std::endl;
								return 1;
				}
				*fetchedValue_ptr = std::to_string(rangeVal);
				return 0;

}

int gphotoWrapper::getToggleConfig(CameraWidget *widget, std::string* fetchedValue_ptr)
{
	char *str;
	int ret = gp_widget_get_value(widget, &str);
	if(ret < GP_OK)
	{
		std::cerr << "Error fetching widget value" << std::endl;
		return 1;
	}
	*fetchedValue_ptr = std::string(str);
	return 0;
}

int gphotoWrapper::getRadioConfig(CameraWidget *widget, std::string* fetchedValue_ptr)
{
	char *str;
	int ret = gp_widget_get_value(widget, &str);
	if(ret < GP_OK)
	{
		std::cerr << "Error fetching widget value" << std::endl;
		return 1;
	}
	*fetchedValue_ptr = std::string(str);
	return 0;
}

int gphotoWrapper::setRadioConfig(CameraWidget *widget, std::string newValue)
{
				int retVal = 1;
				int gphotoRet;

				//Start by building a list of the allowed values
				std::vector<std::string> allowedValues;

				//	Determine the number of choices in the menu
				int Nchoices = gp_widget_count_choices(widget);

				//	Put those choices into a vector
				const char* choice;
				for (int i = 0; i < Nchoices; i++)
				{
								gphotoRet = gp_widget_get_choice(widget, i, &choice);
								if(gphotoRet < GP_OK)
								{
												std::cerr << "ERROR: Failed to get config widget choice" << std::endl;
												//TODO: Process error here
												break;
								}
								allowedValues.push_back(std::string(choice));
				}

				//Check whether the requested value is allowed
				bool demandValid = false;
				for (int i = 0; i < allowedValues.size();i++)
				{
								if (allowedValues[i] == newValue)
								{
												demandValid = true;
												break;
								}
				}

				//Either apply new value, or return error
				if (demandValid == false)
				{
								std::cerr << "ERROR: Requsted value for menu widget not allowed" << std::endl;
								//TODO: Some thought needs to be put in to how to report allowed values back to the caller.
								//I think eventually I will break this into two functions, one to get the allowed values
								//and one to apply the change and it will be up to th caller to decide if its a valid
								//value or not.
								std::cerr << "Allowed values are: ";
								for (int i = 0; i < allowedValues.size()-1; i++)
								{
												std::cerr << allowedValues[i] << ", ";
								}
								std::cerr << allowedValues[allowedValues.size()-1] << std::endl;
				}
				else
				{
								const char* demand = newValue.c_str();
								gphotoRet = gp_widget_set_value(widget, demand);
								if(gphotoRet < GP_OK)
								{
												std::cerr << "ERROR: failed to apply setting to camera" << std::endl;
								}
								else
								{
												retVal = 0;
								}
				}

				return retVal;
}
