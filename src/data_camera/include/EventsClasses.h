#include <string>
#include "interfaces/srv/double_status.hpp"
#include "interfaces/srv/double_request.hpp"
#include "interfaces/srv/int_status.hpp"
#include "interfaces/srv/int_request.hpp"
#include "interfaces/srv/string_status.hpp"
#include "interfaces/srv/string_request.hpp"
#include "interfaces/action/sequence.hpp"

class DataCamera; //Forward declaration
class EventRequest
{
public:
  EventRequest();   //TODO: Default constructor to allow dereived class to work. I feel like this should not be needed. Look into this
  EventRequest(int, DataCamera *);
  ~EventRequest();

    // Overload < operator to allow sorting of event queue
  bool operator < (const EventRequest &);

  int priority;
  std::string timestamp;
  std::string ID;
  bool complete;
  DataCamera * cameranode;
  virtual void execute() = 0;
};

class batteryRequest: public EventRequest
{
public:
  batteryRequest(int, std::shared_ptr < interfaces::srv::IntStatus::Response >, DataCamera *);
  ~batteryRequest();
  void execute() override;
private:
  std::shared_ptr < interfaces::srv::IntStatus::Response > response;
};

class getIsoRequest: public EventRequest
{
public:
  getIsoRequest(int, std::shared_ptr < interfaces::srv::IntStatus::Response >, DataCamera *);
  ~getIsoRequest();
  void execute() override;
private:
  std::shared_ptr < interfaces::srv::IntStatus::Response > response;
};

class setIsoRequest: public EventRequest
{
public:
  setIsoRequest(int, std::shared_ptr < interfaces::srv::IntRequest::Request >,
    std::shared_ptr < interfaces::srv::IntRequest::Response > response, DataCamera *);
  ~setIsoRequest();
  void execute() override;
private:
  std::shared_ptr < interfaces::srv::IntRequest::Request > request;
  std::shared_ptr < interfaces::srv::IntRequest::Response > response;
};

class getImgQualityRequest: public EventRequest
{
public:
  getImgQualityRequest(int, std::shared_ptr < interfaces::srv::StringStatus::Response >,
    DataCamera *);
  ~getImgQualityRequest();
  void execute() override;
private:
  std::shared_ptr < interfaces::srv::StringStatus::Response > response;
};

class setImgQualityRequest: public EventRequest
{
public:
  setImgQualityRequest(int, std::shared_ptr < interfaces::srv::StringRequest::Request >,
    std::shared_ptr < interfaces::srv::StringRequest::Response > response, DataCamera *);
  ~setImgQualityRequest();
  void execute() override;
private:
  std::shared_ptr < interfaces::srv::StringRequest::Request > request;
  std::shared_ptr < interfaces::srv::StringRequest::Response > response;
};

class getFNumberRequest: public EventRequest
{
public:
  getFNumberRequest(int, std::shared_ptr < interfaces::srv::StringStatus::Response >,
    DataCamera *);
  ~getFNumberRequest();
  void execute() override;
private:
  std::shared_ptr < interfaces::srv::StringStatus::Response > response;
};

class setFNumberRequest: public EventRequest
{
public:
  setFNumberRequest(int, std::shared_ptr < interfaces::srv::StringRequest::Request >,
    std::shared_ptr < interfaces::srv::StringRequest::Response > response, DataCamera *);
  ~setFNumberRequest();
  void execute() override;
private:
  std::shared_ptr < interfaces::srv::StringRequest::Request > request;
  std::shared_ptr < interfaces::srv::StringRequest::Response > response;
};

class sequencePhotoRequest: public EventRequest
{
  public:
    sequencePhotoRequest(int, int, std::string, std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>>, DataCamera *);
    ~sequencePhotoRequest();
    void execute() override;
  private:
    std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goalHandle;
    int photoNumber;
};

class generateSequence : public EventRequest
{
  public:
    generateSequence(int, std::string, const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>>, DataCamera *);
    ~generateSequence();
    void execute();
  private:
    std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Sequence>> goalHandle;
};