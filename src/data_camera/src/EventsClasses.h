#include <string>

class DataCamera; //Forward declaration
class EventRequest
{
  public:
    EventRequest(); //TODO: Default constructor to allow dereived class to work. I feel like this should not be needed. Look into this
    EventRequest(int, std::string, DataCamera* );

    int priority;
    bool complete;
    bool status;
    DataCamera* cameranode;
    std::string testname;
    std::string result;
    virtual void execute();
};

class batteryRequest : public EventRequest
{
    public:
        batteryRequest(int, std::string, DataCamera*);
    void execute() override;
};