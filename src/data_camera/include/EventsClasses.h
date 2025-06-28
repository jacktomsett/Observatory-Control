#include <string>
#include "interfaces/srv/int_status.hpp"
#include "interfaces/srv/int_request.hpp"

class DataCamera; //Forward declaration
class EventRequest
{
  public:
    EventRequest(); //TODO: Default constructor to allow dereived class to work. I feel like this should not be needed. Look into this
    EventRequest(int, std::string, DataCamera* );
    ~EventRequest();

    // Overload < operator to allow sorting of event queue
    bool operator<(const EventRequest&);

    int priority;
    std::string timestamp;
    bool complete;
    DataCamera* cameranode;
    virtual void execute() = 0;
};

class batteryRequest : public EventRequest
{
    public:
        batteryRequest(int, std::string, std::shared_ptr<interfaces::srv::IntStatus::Response>, DataCamera*);
        ~batteryRequest();
        void execute() override;
    private:
        std::shared_ptr<interfaces::srv::IntStatus::Response> response;
};

class getIsoRequest : public EventRequest
{
    public:
        getIsoRequest(int, std::string, std::shared_ptr<interfaces::srv::IntStatus::Response>, DataCamera*);
        ~getIsoRequest();
        void execute() override;
    private:
        std::shared_ptr<interfaces::srv::IntStatus::Response> response;
};

class setIsoRequest : public EventRequest
{
    public:
        setIsoRequest(int, std::string,std::shared_ptr<interfaces::srv::IntRequest::Request>,std::shared_ptr<interfaces::srv::IntRequest::Response> response,DataCamera*);
        ~setIsoRequest();
        void execute() override;
    private:
        std::shared_ptr<interfaces::srv::IntRequest::Request> request;
        std::shared_ptr<interfaces::srv::IntRequest::Response> response;
};