//main.cpp
//Author Jack Tomsett
#include <ncurses.h>
#include <menu.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/event.hpp"

//Declare some helper functions for ncurses (copied from ncurses tutorials, might end up removing these in future refactoring)
WINDOW *create_newwin(int height, int width, int starty, int startx);
void destroy_win(WINDOW *local_win);
std::string generateStatusString(int batt, double expo, int iso, double focalLength);

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define CTRLD	4

char *menuItems[] = {
	"Change Setting",
	"Take Photo",
	"Request Sequence",
	"Download Photos",
	"Shutdown Node",
	"Exit"
};

//Declare ros node
class SimpleControlInterface : public rclcpp::Node
{
	public:
		SimpleControlInterface(WINDOW* feedWindowptr)
			: Node("simple_control")
		{
			eventFeedWindowptr = feedWindowptr;
			cameraEventSubscriber = this->create_subscription<interfaces::msg::Event>("camera_events",10, std::bind(&SimpleControlInterface::camera_event_topic_callback,this,std::placeholders::_1));
		}

	private:
		rclcpp::Subscription<interfaces::msg::Event>::SharedPtr cameraEventSubscriber;
		void camera_event_topic_callback(const interfaces::msg::Event & msg)
		{
			//Add new event to buffer		//TODO:: This can probably be better implemented as an ncurses menu, but for now I am just trying to get something on screen
			feedBuffer.push_back(msg.event);
			//Check buffer size
			if (feedBuffer.size() > feedBufferMax)
			{
				feedBuffer.erase(feedBuffer.begin());
			}
			//Clear window
			werase(eventFeedWindowptr);
			//Determine which ones to print, and where
			for(int i = 0; i < feedBuffer.size(); i++)
			{
				mvwaddstr(eventFeedWindowptr,i+2,2,feedBuffer[i].c_str());
			}
			wrefresh(eventFeedWindowptr);
			

		}
		WINDOW* eventFeedWindowptr;
		std::vector<std::string> feedBuffer;
		int feedBufferMax = 10;

};








int main(int argc, char * argv[]){
	//Declare some menu variables
	ITEM **top_items;
	int c;
	MENU *top_menu;
	int n_top_choices, i;
	ITEM *cur_top_item;


	initscr();	//Start curses mode
	cbreak();
	noecho();
	keypad(stdscr,TRUE);

	//set up menu
	n_top_choices = ARRAY_SIZE(menuItems);
	top_items = (ITEM **)calloc(n_top_choices + 1, sizeof(ITEM *));

	for(i = 0; i < n_top_choices; i++)
	{
		top_items[i] = new_item(menuItems[i], menuItems[i]);
	}
	top_items[n_top_choices] = (ITEM *)NULL;

	//Declare windows
	WINDOW* statusWin;
	WINDOW* feedWin;
	WINDOW* menuWin;
	WINDOW* warnWin;
	WINDOW* connWin;
	refresh();
	//Window size parameters
	int bannerHeight = 3; //The height of the conn, status and warn windows
	int connWidth = 15;
	int feedWidth = 30;
	statusWin = create_newwin(bannerHeight,COLS,LINES-bannerHeight,0);
	connWin = create_newwin(bannerHeight,connWidth,0,COLS-connWidth);
	warnWin   = create_newwin(bannerHeight,COLS-connWidth,0,0);
	menuWin   = create_newwin(LINES-(2 * bannerHeight),COLS - feedWidth,bannerHeight,0);
	feedWin   = create_newwin(LINES-(2*bannerHeight),COLS-feedWidth,bannerHeight,(COLS-feedWidth)+1);

	//Associate menu to menuWin, should probably create the menu after the windows
	top_menu = new_menu((ITEM **)top_items);
	set_menu_win(top_menu,menuWin);
	set_menu_sub(top_menu,derwin(menuWin,6,38,3,1));
	post_menu(top_menu);
	wrefresh(menuWin);
	//Dummy data:
	int battery = 5;
	double exp = 15;
	int iso = 400;
	double fnumber = 2.2;
	std::string cameraMode = "Aperture";
	std::string model = "Nikon D3500";
	//Populate status bar
	mvwaddstr(statusWin,1,1,generateStatusString(battery,exp,iso,fnumber).c_str());
	wrefresh(statusWin);
	//Populate camera model
	mvwaddstr(connWin,1,1,model.c_str());
	wrefresh(connWin);
	//Populate warnings
	mvwaddstr(warnWin,1,1,"Camera not in Manual mode! SD card nearing capacity! Battery low!");
	wrefresh(warnWin);


	refresh();

	//Spin up ROS node (need to determine exactly where is best for this, just trying to get something on screen for now)
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<SimpleControlInterface>(feedWin));

	getch();	//Wait for user input, just so we can see the screen
	//Clean up
	rclcpp::shutdown();
	free_item(top_items[0]);
	free_item(top_items[1]);
	free_menu(top_menu);
	destroy_win(statusWin);
	destroy_win(connWin);
	destroy_win(warnWin);
	destroy_win(menuWin);
	destroy_win(feedWin);
	endwin();


	return 0;
}

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

std::string generateStatusString(int batt, double expo, int iso, double focalLength) {
	return std::to_string(expo) + "s | ISO" + std::to_string(iso) + " | f/" + std::to_string(focalLength) + "	| " + std::to_string(batt) + "%";
}
