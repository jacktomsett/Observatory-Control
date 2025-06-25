#include <string>

class DataCamera; //Forward declaration
class EventRequest
{
  public:
    EventRequest(); //TODO: Default constructor to allow dereived class to work. I feel like this should not be needed. Look into this
    EventRequest(int, std::string, DataCamera* );
    ~EventRequest();

    int priority;
    bool complete;
    bool status;
    DataCamera* cameranode;
    std::string eventID;
    std::string result;
    virtual void execute() = 0;
};

class batteryRequest : public EventRequest
{
    public:
        batteryRequest(int, std::string, DataCamera*);
        ~batteryRequest();
        void execute() override;
};

class getIsoRequest : public EventRequest
{
    public:
        getIsoRequest(int, std::string, DataCamera*);
        ~getIsoRequest();
        void execute() override;
};

class setIsoRequest : public EventRequest
{
    public:
        setIsoRequest(int, std::string, int, DataCamera*);
        ~setIsoRequest();
        void execute() override;
    private:
        int demand;
};