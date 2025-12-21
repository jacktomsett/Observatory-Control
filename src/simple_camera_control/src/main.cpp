//main.cpp
//Author Jack Tomsett

//ncurses includes
#include <ncurses.h>
#include <menu.h>

//std includes
#include <string>
#include <thread>
#include <chrono>
#include <mutex>

//Regular ROS2 includes
#include "rclcpp/rclcpp.hpp"

//Project ROS2 includes
#include "interfaces/msg/event.hpp"
#include "interfaces/srv/int_request.hpp"
#include "interfaces/srv/int_status.hpp"
#include "interfaces/srv/string_status.hpp"

//Mutexes
std::mutex topicBufferMutex;
std::mutex valueBufferMutex;
std::mutex exitMutex;
std::mutex commandQueueMutex;

//UI Commands
enum UIcommand {
	//Just getting started, absolutely not an exhaustive list
	INIT_NCURSES,
	UPDATE_FEED,
	UPDATE_STATUS_BANNER,
	UPDATE_WARNINGS,
	RESIZE_UI,
	MENU_UP,
	MENU_DOWN,
	MENU_CHOOSE,
	MENU_BACK,
	NONE
};

//FIXME: Some nasty globals. Declared here so that all the top menu item functions can see
//them. Still figuring out exactly how to structure the arguments of the top level menu
//functions (ideally they should be uniform). I am imagining some kind of struct that gets
//passed to each of them. That struct could end up containing this information
int bannerHeight = 3; //The height of the conn, status and warn windows
int connWidth = 15;
int feedWidth = 60;

//Declare struct that will buffer setting values from camera
struct SettingBuffer{
	//TODO: Currently the camera node sends out exposure anf f-number values as strings because
	//I was too lazy to format them to extract the actual values from them. Eventually I will fix
	//that and then this struct will need to be changed, along with the timer callback in the interface
	//node class and the UI function
	int battery;
	std::string expo;
	int iso;
	std::string aper;
};

//Declare struct that will contain all the pointers needed for menu functions to work
//I'm doing this so that every menu function will have an identical prototype. This avoids
//having to implement code for every menu option in the main function. I don't
//like giving the functions uneeded information but I can't see another way to avoid
//it other than global variables. I think there probably is a way and I should research
//design patterns. I think maybe a state machine might be useful but I need to read about it
//One silver lining is that it will make adding new items to the struct easier while i am still
//developing
struct MenuFunctionData{
	/*
	std::shared_ptr<bool> exitFlag;
	std::shared_ptr<WINDOW*> menuWindow;
	std::shared_ptr<WINDOW*> contextWindow;
	std::shared_ptr<WINDOW*> activeWindow;

	std::shared_ptr<MENU*> topMenu;
	std::shared_ptr<MENU*> settingMenu;
	std::shared_ptr<MENU*> sequenceMenu;
	std::shared_ptr<MENU*> activeMenu;
	*/
	bool* exitFlag;
	WINDOW** menuWindowPtr;
	WINDOW** contextWindowPtr;
	WINDOW** activeWindowPtr;
	WINDOW** previousWindowPtr;

	MENU** topMenuPtr;
	MENU** settingMenuPtr;
	MENU** sequenceMenuPtr;
	MENU** activeMenuPtr;
	MENU** previousMenuPtr;
};


//Declare some helper functions for ncurses (copied from ncurses tutorials, might end up removing these in future refactoring)
WINDOW *create_newwin(int height, int width, int starty, int startx);
void destroy_win(WINDOW *local_win);
std::string generateStatusString(int batt, std::string expo, int iso, std::string focalLength);
//Declare functions that correspond to the top level menu items
void triggerSettingMenu(MenuFunctionData*);
void triggerSequenceMenu(MenuFunctionData*);
void triggerShutdown(MenuFunctionData*);






//Declare ros node
class SimpleControlInterface : public rclcpp::Node
{
	public:
		SimpleControlInterface(std::shared_ptr<std::vector<UIcommand>> uiCommQueue)
			: Node("simple_control"),
			  uiCommandQueue(uiCommQueue)
		{
			feedBufferMax = 100; //TODO: Specify this via a ROS2 parameter
			settingBuffer.battery = 0; //-1 signifies an error (like the camera isnt connected to the camera node)
			settingBuffer.expo = "0";
			settingBuffer.iso = 0;
			settingBuffer.aper = "0";
			//Create callback group
			backgroundFetchCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

			//Init subscribers
			rclcpp::SubscriptionOptions eventSubOptions;
			eventSubOptions.callback_group = backgroundFetchCallbackGroup;
			cameraEventSubscriber = this->create_subscription<interfaces::msg::Event>("/camera_events",10, std::bind(&SimpleControlInterface::camera_event_topic_callback,this,std::placeholders::_1));
			//Init clients
			getIsoClient = this->create_client<interfaces::srv::IntStatus>("get_iso");
			while(!getIsoClient->wait_for_service(std::chrono::seconds(1)))
			{
				//TODO: Have not put any thought into how the program will display the fact that the
				//Camera node has not been detected, but this certainly isnt it. It's not scalable (not
				//going to have a loop for every service I add), it only works if the node is spun before
				//ncurses (havent decided if this will be the case or not yet) and its inneficient
				if(!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "client interrupted waiting for service to appear...");
				}
				RCLCPP_INFO(this->get_logger(), "Listening for camera node to annouce iso status service...");
			}
			getBattClient = this->create_client<interfaces::srv::IntStatus>("battery_status");
			while(!getBattClient->wait_for_service(std::chrono::seconds(1)))
			{
				if(!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "client interrupted waiting for service to appear...");
				}
				RCLCPP_INFO(this->get_logger(), "Listening for camera node to annouce battery status service...");
			}
			getExposureClient = this->create_client<interfaces::srv::StringStatus>("get_exposure");
			while(!getExposureClient->wait_for_service(std::chrono::seconds(1)))
			{
				if(!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "client interrupted waiting for service to appear...");
				}
				RCLCPP_INFO(this->get_logger(), "Listening for camera node to annouce exposure status service...");
			}
			getApertureClient = this->create_client<interfaces::srv::StringStatus>("get_f_number");
			while(!getApertureClient->wait_for_service(std::chrono::seconds(1)))
			{
				if(!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "client interrupted waiting for service to appear...");
				}
				RCLCPP_INFO(this->get_logger(), "Listening for camera node to annouce aperture status service...");
			}			
			//Init timers //TODO: Need to investigate timers more, like what happens when it is still processing the last callback when the next one arrives
			//TODO: Also it would be nice if period of the timer was a ros parameter
			//TODO: Definitely need to add a mechanism to cancel timer if the previous callback is still running
			timer_ = this->create_wall_timer(std::chrono::milliseconds(8000), std::bind(&SimpleControlInterface::timerCallback, this),backgroundFetchCallbackGroup);
		}
		void sendPhotoRequest()
		{

		}
		void sendExpRequest(int exp)
		{

		}
		void sendIsoRequest(int iso)
		{

		}
		void sendAperRequest(int aper)
		{

		}
		std::vector<std::string> getFeedBuffer()
		{
			//This might end up being problematic. If the feedbuffer is
			//large then copying it might introduce slowdowns. It may be
			//better to pass by reference but I thought it would be neater
			//to do it this way because then I could contain all the Mutex
			//code in the ROS node and abstract it away from the calling
			//thread. However it might not be worth the performance hit so
			//check back later

			std::vector<std::string> buffer;
			topicBufferMutex.lock();
			buffer = feedBuffer;
			topicBufferMutex.unlock();
			return buffer;
		}
		SettingBuffer getSettingBuffer()
		{
			//Same issue as above, although It shouldnt be nearly as much of
			//an issue since the buffer is much smaller here

			SettingBuffer buffer;
			valueBufferMutex.lock();
			buffer = settingBuffer;
			valueBufferMutex.unlock();
			return buffer;
		}
	private:
		rclcpp::Subscription<interfaces::msg::Event>::SharedPtr cameraEventSubscriber;
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Client<interfaces::srv::IntStatus>::SharedPtr getIsoClient;
		rclcpp::Client<interfaces::srv::IntStatus>::SharedPtr getBattClient;
		rclcpp::Client<interfaces::srv::StringStatus>::SharedPtr getExposureClient;
		rclcpp::Client<interfaces::srv::StringStatus>::SharedPtr getApertureClient;
		rclcpp::CallbackGroup::SharedPtr backgroundFetchCallbackGroup;
		void camera_event_topic_callback(const interfaces::msg::Event & msg)
		{
			//Add new event to buffer
			topicBufferMutex.lock();
			feedBuffer.push_back(msg.event);
			//Check buffer size
			if (feedBuffer.size() > feedBufferMax)
			{
				feedBuffer.erase(feedBuffer.begin());
			}
			topicBufferMutex.unlock();

			//Tell the ui to update the feed window
			commandQueueMutex.lock();
			uiCommandQueue->push_back(UPDATE_FEED);
			commandQueueMutex.unlock();
		}
		void timerCallback()
		{
			//TODO: Refactor this, could be efficient. Currently making multiple identical request objects that could be resued
			//TODO: Remove all auto keywords. I don't like them, it just shows I don't know what is supposed to be there
			//Declare local variables;
			int battery;
			int iso;
			std::string exp;
			std::string ap;

			//Fetch values from camera
					//TODO: Implement this, currently a placeholder
					//Its actually more complicated than it looks, I think callback groups are involved
			
			auto battRequest = std::make_shared<interfaces::srv::IntStatus::Request>();
			auto batt_result_future = getBattClient->async_send_request(battRequest);
			while(batt_result_future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
			{
				//TODO: Rework this
				//I think this loop will check every 10 milliseconds if the service has finished
				//Still trying to understand client calls and callbacks. Obviously we need to hold
				//wait until this service responds until we do anything else. I think this would require
				//a synchronous service call here but I am not sure that is possible with C++. Need to determine
				//which is the correct type of callback group. Also we should have a timeout if the service
				//does not respond and clear the request appropriately. For now we will just assume that is
				//has responded successfully (horrible I know)
			}
			auto batt_response = batt_result_future.get();
			if(batt_response->status == false)
			{
				battery = -1;
			}
			else{
				battery = batt_response->value;
			}

			auto isoRequest = std::make_shared<interfaces::srv::IntStatus::Request>();
			auto iso_result_future = getIsoClient->async_send_request(isoRequest);
			while(iso_result_future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
			{
				//TODO: Rework this
				//I think this loop will check every 10 milliseconds if the service has finished
				//Still trying to understand client calls and callbacks. Obviously we need to hold
				//wait until this service responds until we do anything else. I think this would require
				//a synchronous service call here but I am not sure that is possible with C++. Need to determine
				//which is the correct type of callback group. Also we should have a timeout if the service
				//does not respond and clear the request appropriately. For now we will just assume that is
				//has responded successfully (horrible I know)
			}
			auto iso_response = iso_result_future.get();
			if(iso_response->status == false)
			{
				iso = -1;
			}
			else{
				iso = iso_response->value;
			}

			auto expRequest = std::make_shared<interfaces::srv::StringStatus::Request>();
			auto exp_result_future = getExposureClient->async_send_request(expRequest);
			while(exp_result_future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
			{
				//TODO: Rework this
				//I think this loop will check every 10 milliseconds if the service has finished
				//Still trying to understand client calls and callbacks. Obviously we need to hold
				//wait until this service responds until we do anything else. I think this would require
				//a synchronous service call here but I am not sure that is possible with C++. Need to determine
				//which is the correct type of callback group. Also we should have a timeout if the service
				//does not respond and clear the request appropriately. For now we will just assume that is
				//has responded successfully (horrible I know)
			}
			auto exp_response = exp_result_future.get();
			if(exp_response->status == false)
			{
				exp = "err";
			}
			else{
				exp = exp_response->value;
			}

			auto apRequest = std::make_shared<interfaces::srv::StringStatus::Request>();
			auto ap_result_future = getApertureClient->async_send_request(apRequest);
			while(ap_result_future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
			{
				//TODO: Rework this
				//I think this loop will check every 10 milliseconds if the service has finished
				//Still trying to understand client calls and callbacks. Obviously we need to hold
				//wait until this service responds until we do anything else. I think this would require
				//a synchronous service call here but I am not sure that is possible with C++. Need to determine
				//which is the correct type of callback group. Also we should have a timeout if the service
				//does not respond and clear the request appropriately. For now we will just assume that is
				//has responded successfully (horrible I know)
			}
			auto ap_response = ap_result_future.get();
			if(ap_response->status == false)
			{
				ap = "err";
			}
			else{
				ap = ap_response->value;
			}

			//Update buffer values
			valueBufferMutex.lock();

			settingBuffer.battery = battery;
			settingBuffer.iso = iso;
			settingBuffer.expo = exp;
			settingBuffer.aper = ap;

			valueBufferMutex.unlock();

			//Tell the ui to update the status window
			commandQueueMutex.lock();
			uiCommandQueue->push_back(UPDATE_STATUS_BANNER);
			commandQueueMutex.unlock();
		};

		std::shared_ptr<std::vector<UIcommand>> uiCommandQueue;
		std::vector<std::string> feedBuffer;
		int feedBufferMax;
		SettingBuffer settingBuffer;
};


//Declare struct that will contain everything ui thread needs
struct UiData{
    std::shared_ptr<bool> exitFlag;
		std::shared_ptr<std::vector<UIcommand>> commandQueue;
		std::shared_ptr<SimpleControlInterface> node;
};
void uiThreadFunction(UiData data);



WINDOW *create_newwin(int height, int width, int starty, int startx){
	WINDOW *local_win;
	local_win = newwin(height, width, starty, startx);
	box(local_win, 0, 0); //0,0 specifiec the default characters for the box
	wrefresh(local_win); //Show the box
	return local_win;
}

void destroy_win(WINDOW *local_win){
	// the function box(local_win, ' ',' ') Would not clear the
	// the border as expected, it would leave the four corners
	// behind. Instead, do the following...
	
	wborder(local_win, ' ',' ',' ',' ',' ',' ',' ',' ');
	//The parameters are: window, left side, right side,
	//top side, bottom side, top left, top right, bottom left
	//bottom right
	wrefresh(local_win);
	delwin(local_win);

	return;
}

std::string generateStatusString(int batt, std::string expo, int iso, std::string focalLength) {
	std::string battString,expoString,isoString,focalLengthString;
	if(batt < 0)
	{
		battString = "-";
	}
	else
	{
		battString = std::to_string(batt) + "%";
	}
	if(expo == "err")
	{
		expoString = "N/A";
	}
	else
	{
		expoString = expo;
	}
	if(iso < 0)
	{
		isoString = "-";
	}
	else
	{
		isoString = "ISO " + std::to_string(iso);
	}
	if(focalLength == "err")
	{
		focalLengthString = "N/A";
	}
	else
	{
		focalLengthString = focalLength;
	}
	return expoString + " | ISO " + isoString + " | " + focalLengthString + "	| " + battString ;
}

//Define top level menu functions
void triggerSettingMenu(MenuFunctionData *menuFunctionParameters){ //I would have thought this could be acheived by passing the struct by reference, but I couldn't get it to work.
																   //Although now I think about it, if we are passing the struct as a pointer then there is no need to have the struct members
																   //also be pointers. i only made them pointers in the first place to minimise the amount of data to copy.
	post_menu(*((*menuFunctionParameters).settingMenuPtr));
	wrefresh(*((*menuFunctionParameters).contextWindowPtr));
	(*menuFunctionParameters).activeWindowPtr = (*menuFunctionParameters).contextWindowPtr;
	(*menuFunctionParameters).activeMenuPtr = (*menuFunctionParameters).settingMenuPtr;
	(*menuFunctionParameters).previousWindowPtr = (*menuFunctionParameters).menuWindowPtr;
	(*menuFunctionParameters).previousMenuPtr = (*menuFunctionParameters).topMenuPtr;
}
void triggerSequenceMenu(MenuFunctionData *menuFunctionParameters){
	post_menu(*((*menuFunctionParameters).sequenceMenuPtr));
	wrefresh(*((*menuFunctionParameters).contextWindowPtr));
	(*menuFunctionParameters).activeWindowPtr = (*menuFunctionParameters).contextWindowPtr;
	(*menuFunctionParameters).activeMenuPtr = (*menuFunctionParameters).sequenceMenuPtr;
	(*menuFunctionParameters).previousWindowPtr = (*menuFunctionParameters).menuWindowPtr;
	(*menuFunctionParameters).previousMenuPtr = (*menuFunctionParameters).topMenuPtr;
}
void triggerShutdown(MenuFunctionData *pointers){
	*((*pointers).exitFlag) = true;
}

void backgroundThreadFunction(SimpleControlInterface node)
{
	//Spin node to check for new subscription messages
	//TODO: Move the lambda in the main function to here
	//TODO: Rename function to nodeThreadFunction when you do
	//Check 
}

void updateFeedWindow(std::shared_ptr<std::vector<std::string>> buffer, std::shared_ptr<SimpleControlInterface> node, std::shared_ptr<WINDOW*> win, std::shared_ptr<MENU*> menu, std::shared_ptr<ITEM **> items)
{
	unpost_menu(*menu);
	free_menu(*menu);
	for(int i = 0; i < buffer->size(); i++)
	{
		free_item((*items)[i]);
	}
				
	*buffer = node->getFeedBuffer();
				
	*items = (ITEM **)calloc(buffer->size()+1, sizeof(ITEM *));
	for(int i = 0; i < buffer->size(); i++)
	{
		(*items)[i] = new_item((*(buffer))[i].c_str(),"");
	}
	*menu = new_menu((ITEM **)(*items));
	set_menu_win(*menu,*win);
	set_menu_sub(*menu,derwin(*win,LINES-(2*bannerHeight)-4,(COLS-feedWidth-2)-2,1,1));
	set_menu_mark(*menu,"");
	post_menu(*menu);

	menu_driver(*menu,REQ_LAST_ITEM);
	wrefresh(*win);
	refresh();
	
}
void updateStatusWindow(std::shared_ptr<SettingBuffer> buffer, std::shared_ptr<SimpleControlInterface> node, std::shared_ptr<WINDOW*> win)
{
	//FIXME: Need to make sure the previous contents of the window are cleared first
	*buffer = node->getSettingBuffer();
	mvwaddstr(*win,1,1,generateStatusString((*buffer).battery,(*buffer).expo,(*buffer).iso,(*buffer).aper).c_str());
	wrefresh(*win);
}


int main(int argc, char * argv[]){

	//Spin up ROS node (need to determine exactly where is best for this, just trying to get something on screen for now)
	rclcpp::init(argc,argv);
	std::shared_ptr<std::vector<UIcommand>> uiCommandQueue = std::make_shared<std::vector<UIcommand>>();
	std::shared_ptr<SimpleControlInterface> nodeHandle = std::make_shared<SimpleControlInterface>(uiCommandQueue);
	
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(nodeHandle);
	std::thread nodeThread([&executor](){executor.spin();}); //TODO: Move this lambda into its own (alread declared function). Ensure proper shutdown handling occurs
  
	
	//std::shared_ptr<bool> exitFlag = std::make_shared<bool>(false); //FIXME: Now we have gone back to two threads, this doesnt need to be a pointer anymore


//////////////////////////////////////////////////////////////////
  	//TODO: Fill out barebones template along the lines of notebook. Just get the exit flag working and shutdown working first
  	//and fix ros node shutdown too while you're at it
	enum UIcommand currentCommand;
	std::shared_ptr<std::vector<std::string>> feedBufferPtr = std::make_shared<std::vector<std::string>>();
	std::shared_ptr<SettingBuffer> statusBufferPtr = std::make_shared<SettingBuffer>();
	MenuFunctionData menuFunctionParameters;
	bool shouldExit = false;
	menuFunctionParameters.exitFlag = &shouldExit;

	//Initialise ncurses here. Debating whether this should be the first command in the queue,
	//depends on if there is ever a situation where you would not want to draw the ui at startup.
	initscr();	//Start curses mode
	cbreak();
	noecho();
	nodelay(stdscr,TRUE); //Should make getch() non blocking
	keypad(stdscr,TRUE);

	//Declare windows
	std::shared_ptr<WINDOW*> statusWinPtr = std::make_shared<WINDOW*>();
	std::shared_ptr<WINDOW*> feedWinPtr = std::make_shared<WINDOW*>();
	//std::shared_ptr<WINDOW*> menuWinPtr = std::make_shared<WINDOW*>();
	//menuFunctionParameters.menuWindow = menuWinPtr;
	//menuFunctionParameters.menuWindow = std::make_shared<WINDOW*>();
	WINDOW* menuWindow;
	menuFunctionParameters.menuWindowPtr = &menuWindow;
	std::shared_ptr<WINDOW*> warnWinPtr = std::make_shared<WINDOW*>();
	std::shared_ptr<WINDOW*> connWinPtr = std::make_shared<WINDOW*>();
	//std::shared_ptr<WINDOW*> contextWinPtr = std::make_shared<WINDOW*>();
	//menuFunctionParameters.contextWindow = contextWinPtr;
	//menuFunctionParameters.contextWindow = std::make_shared<WINDOW*>();
	WINDOW* contextWindow;
	menuFunctionParameters.contextWindowPtr = &contextWindow;
	menuFunctionParameters.activeWindowPtr = menuFunctionParameters.menuWindowPtr;
	menuFunctionParameters.previousWindowPtr = menuFunctionParameters.menuWindowPtr;

	refresh();

	*statusWinPtr = create_newwin(bannerHeight,COLS,LINES-bannerHeight,0);
	*connWinPtr = create_newwin(bannerHeight,connWidth,0,COLS-connWidth);
	*warnWinPtr   = create_newwin(bannerHeight,COLS-connWidth,0,0);
	//*menuWinPtr   = create_newwin(LINES-(2 * bannerHeight),(COLS - feedWidth-1)/2,bannerHeight,0);
	*(menuFunctionParameters.menuWindowPtr)   = create_newwin(LINES-(2 * bannerHeight),(COLS - feedWidth-1)/2,bannerHeight,0);
	//*contextWinPtr   = create_newwin(LINES-(2 * bannerHeight),(COLS - feedWidth-1)/2,bannerHeight,(COLS - feedWidth-1)/2);
	*(menuFunctionParameters.contextWindowPtr)   = create_newwin(LINES-(2 * bannerHeight),(COLS - feedWidth-1)/2,bannerHeight,(COLS - feedWidth-1)/2);
	*feedWinPtr   = create_newwin(LINES-(2*bannerHeight),COLS-feedWidth-1,bannerHeight,(COLS-feedWidth));
	
	//nodelay(*menuWinPtr,TRUE);
	nodelay(*(menuFunctionParameters.menuWindowPtr),TRUE);

	//========  Set up menus
	//Top level menu
	std::vector<std::string>topMenuItems = {
	"Change Setting",
	"Request Sequence",
	"Download Photos",
	"Shutdown Node",
	"Exit"
	};

	ITEM **top_items;
	top_items = (ITEM **)calloc(topMenuItems.size()+1, sizeof(ITEM *));
	for(int i = 0; i < topMenuItems.size(); i++)
	{
		top_items[i] = new_item(topMenuItems[i].c_str(),"");
	}
	//Set the user pointers
	set_item_userptr(top_items[0],(void*)triggerSettingMenu);
	set_item_userptr(top_items[1],(void*)triggerSequenceMenu);
	set_item_userptr(top_items[topMenuItems.size()-1],(void*)triggerShutdown); //Final menu item, corresponding to exit


	
	//std::shared_ptr<MENU*> top_menuPtr = std::make_shared<MENU*>();
	//*top_menuPtr = new_menu((ITEM **)top_items);
	MENU* topMenu;
	menuFunctionParameters.topMenuPtr = &topMenu;
	*(menuFunctionParameters.topMenuPtr) = new_menu((ITEM **)top_items);
	//set_menu_win(*top_menuPtr,*menuWinPtr);
	set_menu_win(*(menuFunctionParameters.topMenuPtr),*(menuFunctionParameters.menuWindowPtr));
	//set_menu_sub(*top_menuPtr,derwin(*menuWinPtr,LINES-(2 * bannerHeight)-4,((COLS - feedWidth-1)/2)-2,1,1));
	set_menu_sub(*(menuFunctionParameters.topMenuPtr),derwin(*(menuFunctionParameters.menuWindowPtr),LINES-(2 * bannerHeight)-4,((COLS - feedWidth-1)/2)-2,1,1));
	set_menu_mark(*(menuFunctionParameters.topMenuPtr),"");
	post_menu(*(menuFunctionParameters.topMenuPtr));

	//menuFunctionParameters.topMenu = top_menuPtr;

	//Setting menu
	std::vector<std::string>settingMenuItems = {
	"Exposure",
	"ISO",
	"Aperture"
	};

	ITEM **setting_items;
	setting_items = (ITEM **)calloc(settingMenuItems.size()+1, sizeof(ITEM *));
	for(int i = 0; i < settingMenuItems.size(); i++)
	{
		setting_items[i] = new_item(settingMenuItems[i].c_str(),"");
	}
	//TODO: Set the user pointers

	MENU* settingMenu;
	menuFunctionParameters.settingMenuPtr = &settingMenu;
	*(menuFunctionParameters.settingMenuPtr) = new_menu((ITEM **)setting_items);
	set_menu_win(*(menuFunctionParameters.settingMenuPtr),*(menuFunctionParameters.contextWindowPtr));
	set_menu_sub(*(menuFunctionParameters.settingMenuPtr),derwin(*(menuFunctionParameters.contextWindowPtr),LINES-(2 * bannerHeight)-4,((COLS - feedWidth-1)/2)-2,1,1));
	set_menu_mark(*(menuFunctionParameters.settingMenuPtr),"");

	//Sequence menu
	std::vector<std::string>sequenceMenuItems = {
	"Length",
	"Name",
	"StartSequence"
	};

	ITEM **sequence_items;
	sequence_items = (ITEM **)calloc(sequenceMenuItems.size()+1, sizeof(ITEM *));
	for(int i = 0; i < sequenceMenuItems.size(); i++)
	{
		sequence_items[i] = new_item(sequenceMenuItems[i].c_str(),"");
	}
	//TODO: Set the user pointers

	MENU* sequenceMenu;
	menuFunctionParameters.sequenceMenuPtr = &sequenceMenu;
	*(menuFunctionParameters.sequenceMenuPtr) = new_menu((ITEM **)sequence_items);
	set_menu_win(*(menuFunctionParameters.sequenceMenuPtr),*(menuFunctionParameters.contextWindowPtr));
	set_menu_sub(*(menuFunctionParameters.sequenceMenuPtr),derwin(*(menuFunctionParameters.contextWindowPtr),LINES-(2 * bannerHeight)-4,((COLS - feedWidth-1)/2)-2,1,1));
	set_menu_mark(*(menuFunctionParameters.sequenceMenuPtr),"");

	//Set the active menu
	menuFunctionParameters.activeMenuPtr = menuFunctionParameters.topMenuPtr;

	//wrefresh(*menuWinPtr);
	wrefresh(*(menuFunctionParameters.menuWindowPtr));
	//wrefresh(*contextWinPtr);
	wrefresh(*(menuFunctionParameters.contextWindowPtr));
	

	std::string cameraMode = "Aperture";
	std::string model = "Nikon D3500";
	//Populate status bar //TODO: Implement: Following two lines copied from previous iteration. I have not given the node thread access to the setting value buffer yet (or maybe just access to the node would be enough if the getter functions were wrapped with mutexes)
	//mvwaddstr(statusWin,1,1,generateStatusString(cameraValues.battery,cameraValues.expo,cameraValues.iso,cameraValues.aper).c_str());
	//wrefresh(statusWin);
	//Populate camera model
	mvwaddstr(*connWinPtr,1,1,model.c_str());
	wrefresh(*connWinPtr);
	//Populate warnings
	mvwaddstr(*warnWinPtr,1,1,"Camera not in Manual mode! SD card nearing capacity! Battery low!");
	wrefresh(*warnWinPtr);

	//TODO: This, and the the function to update the feed menu only worked when creating a pointer to every ncurses variable, and acting upon those
	//pointers here. When I acted on the raw ncurses objects here it didnt work and made a whole separate window for the menu created in the function
	//as opposed to the window here. This suggests that when invoking the make_shared function it is creating a new underlying variable. If this is indeed
	//the case then the code can probably be cleaned up here by not bothering to create the raw ncurses variables at all and just instantiate the smart pointers
	//and always act upon them. Read the documentation on the make_shared (and similar) functions first to properly understand them.
	MENU *feed_menu; //holds the camera event feed menu. The menu itself gets created and destroyed on each loop iteration
	std::shared_ptr<MENU*> feed_menu_ptr = std::make_shared<MENU*>(feed_menu);
	*feedBufferPtr = nodeHandle->getFeedBuffer();
	std::shared_ptr<ITEM**> feed_items_ptr = std::make_shared<ITEM**>();
	*feed_items_ptr = (ITEM **)calloc(feedBufferPtr->size()+1, sizeof(ITEM *));
	for(int i = 0; i < feedBufferPtr->size(); i++)
	{
		(*feed_items_ptr)[i] = new_item((*feedBufferPtr)[i].c_str(),"");
	}
	*feed_menu_ptr = new_menu((ITEM **)(*feed_items_ptr));
	set_menu_win(*feed_menu_ptr,*feedWinPtr);
	set_menu_sub(*feed_menu_ptr,derwin(*feedWinPtr,LINES-(2*bannerHeight)-4,(COLS-feedWidth-2)-2,1,1));
	set_menu_mark(*feed_menu_ptr,"");
	post_menu(*feed_menu_ptr);
	wrefresh(*feedWinPtr);
	
	//Set up status window
	*statusBufferPtr = nodeHandle->getSettingBuffer();
	mvwaddstr(*statusWinPtr,1,1,generateStatusString((*statusBufferPtr).battery,(*statusBufferPtr).expo,(*statusBufferPtr).iso,(*statusBufferPtr).aper).c_str());
	wrefresh(*statusWinPtr);

	refresh();
	int c;
  	while (shouldExit == false)
  	{

			//Get user input
			
			c = getch();
			switch(c)
			{
				//TODO: Refactor into a function
				case 'j' :
					commandQueueMutex.lock();
					uiCommandQueue->push_back(MENU_DOWN);
					commandQueueMutex.unlock();
					break;
				case 'k' :
					commandQueueMutex.lock();
					uiCommandQueue->push_back(MENU_UP);
					commandQueueMutex.unlock();
					break;
				case 'e' :
					commandQueueMutex.lock();
					uiCommandQueue->push_back(MENU_CHOOSE);
					commandQueueMutex.unlock();
					break;
				case 'b' :
					commandQueueMutex.lock();
					uiCommandQueue->push_back(MENU_BACK);
					commandQueueMutex.unlock();
					break;
			}

			//Extract command from queue
			commandQueueMutex.lock();
			if(uiCommandQueue->empty() == 1)
			{
				commandQueueMutex.unlock();
				currentCommand = NONE;
			}
			else
			{
				currentCommand = (*uiCommandQueue)[0];
				uiCommandQueue->erase(uiCommandQueue->begin());
				commandQueueMutex.unlock();
			}

			switch(currentCommand)
			{
				case NONE :
					break;
				case UPDATE_FEED :
					updateFeedWindow(feedBufferPtr,nodeHandle,feedWinPtr,feed_menu_ptr,feed_items_ptr);
					break;
				case UPDATE_STATUS_BANNER :
					updateStatusWindow(statusBufferPtr,nodeHandle,statusWinPtr);
					break;
				case MENU_UP :
					menu_driver(*(menuFunctionParameters.activeMenuPtr), REQ_UP_ITEM);
					wrefresh(*(menuFunctionParameters.activeWindowPtr));
					break;
				case MENU_DOWN :
					menu_driver(*(menuFunctionParameters.activeMenuPtr), REQ_DOWN_ITEM);
					wrefresh(*(menuFunctionParameters.activeWindowPtr));
					break;
				case MENU_CHOOSE :
					//TODO: Should check for void pointer here. Also put into function
					ITEM *cur;
					void (*p)(MenuFunctionData*);
					cur = current_item(*(menuFunctionParameters.activeMenuPtr));
					p = reinterpret_cast<void (*)(MenuFunctionData*)>(item_userptr(cur));
					p(&menuFunctionParameters);
					break;
				case MENU_BACK :
					//TODO: Put into function
					if(menuFunctionParameters.activeMenuPtr != menuFunctionParameters.topMenuPtr)
					{
						unpost_menu(*(menuFunctionParameters.activeMenuPtr));
						menuFunctionParameters.activeMenuPtr = menuFunctionParameters.previousMenuPtr;
						menuFunctionParameters.activeWindowPtr = menuFunctionParameters.previousWindowPtr;
					}
					
				default:
					//Eventually need to throw some sort of error here once I have implemented all the commands
					//and an error system
					break;
			}

  	}



/////////////////////////////////////////////////////////
	//Clean up
	rclcpp::shutdown();
  	//Shutdown ncurses
  	unpost_menu(feed_menu);
	free_menu(feed_menu);
	for(int i = 0; i < feedBufferPtr->size(); i++)
	{
		free_item((*feed_items_ptr)[i]);
	}
	unpost_menu(*(menuFunctionParameters.topMenuPtr));
	free_menu(*(menuFunctionParameters.topMenuPtr));
	for(int i = 0; i < topMenuItems.size(); i++)
	{
		free_item(top_items[i]);
	}
	destroy_win(*statusWinPtr);
	destroy_win(*connWinPtr);
	destroy_win(*warnWinPtr);
	//destroy_win(*menuWinPtr);
	destroy_win(*(menuFunctionParameters.menuWindowPtr));
	destroy_win(*feedWinPtr);
	//destroy_win(*contextWinPtr);
	destroy_win(*(menuFunctionParameters.contextWindowPtr));
	endwin();
	nodeThread.join();

	return 0;
}