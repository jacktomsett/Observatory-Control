#include <string.h>
#include <string>
#include <iostream>
#include <gphoto2/gphoto2-port-result.h>
#include <gphoto2/gphoto2-result.h>



//Following functions copied from libgphoto samples from github. They seem to be gphotos error log system. Eventually
//need to figure out how to get it to write to ros info
static void
ctx_error_func (GPContext *context, const char *str, void *data)
{
        fprintf  (stderr, "\n*** Contexterror ***              \n%s\n",str);
        fflush   (stderr);
}

static void
ctx_status_func (GPContext *context, const char *str, void *data)
{
        fprintf  (stderr, "%s\n", str);
        fflush   (stderr);
}
static void
ctx_message_func (GPContext *context, const char *str, void *data)
{
        std::string message(str);
		std::cout << message << std::endl;
}
GPContext* sample_create_context() {
	GPContext *context;

	/* This is the mandatory part */
	context = gp_context_new();

	/* All the parts below are optional! */
        gp_context_set_error_func (context, ctx_error_func, NULL);
        gp_context_set_status_func (context, ctx_status_func, NULL);
		gp_context_set_message_func   (context, ctx_message_func, NULL);

	/* also:
	gp_context_set_cancel_func    (p->context, ctx_cancel_func,  p);
        gp_context_set_message_func   (p->context, ctx_message_func, p);
        if (isatty (STDOUT_FILENO))
                gp_context_set_progress_funcs (p->context,
                        ctx_progress_start_func, ctx_progress_update_func,
                        ctx_progress_stop_func, p);
	 */
	return context;
}

static int
_lookup_widget(CameraWidget*widget, const char *key, CameraWidget **child) {
	int ret;
	ret = gp_widget_get_child_by_name (widget, key, child);
	if (ret < GP_OK)
		ret = gp_widget_get_child_by_label (widget, key, child);
	return ret;
}

/* Gets a string configuration value.
 * This can be:
 *  - A Text widget
 *  - The current selection of a Radio Button choice
 *  - The current selection of a Menu choice
 *
 * Sample (for Canons eg):
 *   get_config_value_string (camera, "owner", &ownerstr, context);
 */
int
get_config_value_string (Camera *camera, const char *key, char **str, GPContext *context) {
	CameraWidget		*widget = NULL, *child = NULL;
	CameraWidgetType	type;
	int			ret;
	char			*val;

	ret = gp_camera_get_single_config (camera, key, &child, context);
	if (ret == GP_OK) {
		if (!child) std::cout << "child is NULL?" << std::endl;;
		widget = child;
	} else {
		ret = gp_camera_get_config (camera, &widget, context);
		if (ret < GP_OK) {
			std::cout << "camera_get_config failed: " << ret << std::endl;
			return ret;
		}
		ret = _lookup_widget (widget, key, &child);
		if (ret < GP_OK) {
			std::cout << "lookup widget failed: " << ret << std::endl;
			return ret;
		}
	}
	
	/* This type check is optional, if you know what type the label
	 * has already. If you are not sure, better check. */
	/*
	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		fprintf (stderr, "widget get type failed: %d\n", ret);
		return ret;
	}
	switch (type) {
        case GP_WIDGET_MENU:
        case GP_WIDGET_RADIO:
        case GP_WIDGET_TEXT:
		break;
	default:
		fprintf (stderr, "widget has bad type %d\n", type);
		ret = GP_ERROR_BAD_PARAMETERS;
		return ret;
	}
	*/
	/* This is the actual query call. Note that we just
	 * a pointer reference to the string, not a copy... */
	ret = gp_widget_get_value (child, &val);
	if (ret < GP_OK) {
		std::cout << "could not query widget value: " << ret << std::endl;
		return ret;
	}
	/* Create a new copy for our caller. */
	*str = strdup (val);
	return ret;
}


/* Sets a string configuration value.
 * This can set for:
 *  - A Text widget
 *  - The current selection of a Radio Button choice
 *  - The current selection of a Menu choice
 *
 * Sample (for Canons eg):
 *   get_config_value_string (camera, "owner", &ownerstr, context);
 */
int
set_config_value_string (Camera *camera, const char *key, const char *val, GPContext *context) {
	CameraWidget		*widget = NULL, *child = NULL;
	CameraWidgetType	type;
	int			ret;

	ret = gp_camera_get_config (camera, &widget, context);
	//std::cout << "Get config return value: " << ret << std::endl;
	if (ret < GP_OK) {
		std::cout << "camera_get_config failed: " << ret << std::endl;
		return ret;
	}
	ret = _lookup_widget (widget, key, &child);
	if (ret < GP_OK) {
		std::cout <<  "lookup widget failed: " << ret << std::endl;
		return ret;
	}
	/* This type check is optional, if you know what type the label
	 * has already. If you are not sure, better check. */
	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		std::cout <<  "widget get type failed: " << ret << std::endl;
		return ret;
	}
	switch (type) {
        case GP_WIDGET_MENU:
			std::cout << "Widget type: Menu" << std::endl;
			break;
        case GP_WIDGET_RADIO:
			std::cout << "Widget type: Radio" << std::endl;
			break;
        case GP_WIDGET_TEXT:
			std::cout << "Widget type: Menu" << std::endl;
			break;
		break;
	default:
		std::cout <<  "widget has bad type: " << ret << std::endl;
		ret = GP_ERROR_BAD_PARAMETERS;
		return ret;
	}

	/* This is the actual set call. Note that we keep
	 * ownership of the string and have to free it if necessary.
	 */
	ret = gp_widget_set_value (child, val);
	//std::cout << "gp_widget_set_value() returned error code: " << ret <<std::endl;
	if (ret < GP_OK) {
		//std::cout <<  "could not set widget value: " << ret << std::endl;
		return ret;
	}
	ret = gp_camera_set_single_config (camera, key, child, context);
	//std::cout << "gp_camera_set_single_config() returned error code: " << ret <<std::endl;
	//if (ret != GP_OK) {
		/* This stores it on the camera again */
		//ret = gp_camera_set_config (camera, widget, context);
		//std::cout << "gp_camera_set_config() returned error code: " << ret <<std::endl;
		//if (ret < GP_OK) {
		//	std::cout <<  "camera_set_config failed: " << ret << std::endl;
		//	return ret;
		//}
		//else{
		//	return ret;
		//}
	//}

	return ret;
}



/*
std::string get_error_as_string(int gp_error){
	const char * result = gp_port_result_as_string(gp_error);
	std::string result_string(result);
	return result_string;
}
*/