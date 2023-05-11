#include <iostream>
#include <string>
#include <curl/curl.h>
#include <astronomyapi_interface.cpp>
#include <jsoncpp/json/json.h>
#include <vector>

int main()
{
    // api credentials...
    std::string AppID = "c2a53679-543a-400f-8f03-625e762aab91";
    std::string AppSecretKey = "195d8f3ab1f3c5cf267b810cb34368a11a16e043ecd5edb8bf45e36b8c199b0aa25471a56f975c0e71c560fbebf5f7d62a3bde4316748a28b1ce59ca11750add9a1eb7c5a8e7437def8c5ad5e5ec49dd4eccd87a889238d5b04eb4daa0210d367af30feaa6a6c5b27c7b890b9e35432e";

    // obvserver data...
    ObserverParams Whiston;
    Whiston.Lattitude = 53.412791;
    Whiston.Longitude = 2.801705;
    Whiston.Elevation = 1;
    Whiston.StartDate = Date(2023,5,10);
    Whiston.EndDate = Date(2023,05,10);
    Whiston.time = "21:30:00";

    std::string target = "jupiter";
    std::string apiresponse = getBodyPosition(target,Whiston,generateAuthString(AppID,AppSecretKey));
    std::cout << apiresponse << std::endl;
    Json::Reader reader;
    Json::Value Response;
    reader.parse(apiresponse,Response);
    //Json::Value Response = Response["data"];
    //Json::Value Response = Response["table"];
    //Json::Value namedata = Response["rows"];
    //std::cout << "id: " << namedata["id"] << std::endl;
    //std::cout << "name: " << namedata["name"] << std::endl;
    //std::cout << namedata << std:: endl;

    std::vector<std::string> memberNames = Response["data"]["table"]["rows"][1]["entry"].getMemberNames();
    int numberofrows = Response["data"]["table"]["rows"][1]["cells"].size();
    std::string name = Response["data"]["table"]["rows"][1]["entry"]["name"].asString();
    std::cout << "id:            " << Response["data"]["table"]["rows"][1]["cells"][1]["id"].asString() << std::endl;
    std::cout << "name:          " << name << std::endl;
    std::cout << "RA:            " << Response["data"]["table"]["rows"][1]["cells"][1]["position"]["equatorial"]["rightAscension"]["string"] << std::endl;
    std::cout << "Dec:           " << Response["data"]["table"]["rows"][1]["cells"][1]["declination"]["equatorial"]["rightAscension"]["string"] << std::endl;
    std::cout << "Constellation: " << Response["data"]["table"]["rows"][1]["cells"][1]["position"]["constellation"]["name"] << std::endl;


    return 0;
}