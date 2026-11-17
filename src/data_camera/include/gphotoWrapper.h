#ifndef GPHOTOWRAPPER_H
#define GPHOTOWRAPPER_H

#include "gphoto2/gphoto2.h"
#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2-context.h>
#include <gphoto2/gphoto2-widget.h>
#include <rclcpp/logger.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

//forward declaration
class DataCamera;

class gphotoWrapper
{
	private:
	Camera *camera;
	GPContext *context;
	CameraWidget *topLevelWidget;
	std::string serialNumber;
	bool cameraPresentFlag;
	int disconnectCamera;
	static void ctx_error_func(GPContext *context, const char *str, void *data);
	static void ctx_status_func(GPContext *context, const char *str, void *data);
	DataCamera* node_ptr;
	

	//Sub functions to get and set config values. Will be called by the public functions as appropriate
	int getTextConfig(CameraWidget *widget, std::string* str_ptr);
	int getRangeConfig(CameraWidget *widget, std::string* str_ptr);
	int getToggleConfig(CameraWidget *widget, std::string* str_ptr);
	int getRadioConfig(CameraWidget *widget, std::string* str_ptr);
	int getMenuConfig(CameraWidget *widget, std::string* str_ptr);
	int getButtonConfig(CameraWidget *widget, std::string* str_ptr);
	int getDateConfig(CameraWidget *widget, std::string* str_ptr);

	int setTextConfig(CameraWidget *widget, std::string);
	int setRangeConfig(CameraWidget *widget, std::string);
	int setToggleConfig(CameraWidget *widget, std::string);
	int setRadioConfig(CameraWidget *widget, std::string);
	int setMenuConfig(CameraWidget *widget, std::string);
	int setButtonConfig(CameraWidget *widget, std::string);
	int setDateConfig(CameraWidget *widget, std::string);

	public:
		gphotoWrapper(DataCamera*);
		~gphotoWrapper();
		bool isCameraPresent();
		int checkConnection();
		int getSettingValue(std::string parameter, std::string* value_ptr);
		int setSettingValue(std::string parameter, std::string newValue);
		int captureImage();

};

#endif
