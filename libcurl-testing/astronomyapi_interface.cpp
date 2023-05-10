#include <string>
#include <curl/curl.h>
#include <Date.h>
#include <Date.cpp>

//Define observer paramater structure
struct ObserverParams
{
    float Longitude;
    float Lattitude;
    int Elevation;
    Date StartDate;
    Date EndDate;
    std::string time;
};

std::string buildQueryString(ObserverParams observer)
{
    //convert :'s in time string
    std::string timestring = observer.time;
    timestring.replace(3,1,"%3A");
    timestring.replace(7,1,"%3A");
    return "?longitude=" + std::to_string(observer.Longitude) + "&lattitude=" + std::to_string(observer.Lattitude) + "&elevation=" + std::to_string(observer.Elevation) + "&from_date=" + observer.StartDate.print() + "&to_date=" + observer.EndDate.print() + "&time=" + timestring;

}

//=========To Do.....
std::string generateAuthString(std::string ApplicationID, std::string ApplicationSecret)
{

    return "Authorization: Basic YzJhNTM2NzktNTQzYS00MDBmLThmMDMtNjI1ZTc2MmFhYjkxOjE5NWQ4ZjNhYjFmM2M1Y2YyNjdiODEwY2IzNDM2OGExMWExNmUwNDNlY2Q1ZWRiOGJmNDVlMzZiOGMxOTliMGFhMjU0NzFhNTZmOTc1YzBlNzFjNTYwZmJlYmY1ZjdkNjJhM2JkZTQzMTY3NDhhMjhiMWNlNTljYTExNzUwYWRkOWExZWI3YzVhOGU3NDM3ZGVmOGM1YWQ1ZTVlYzQ5ZGQ0ZWNjZDg3YTg4OTIzOGQ1YjA0ZWI0ZGFhMDIxMGQzNjdhZjMwZmVhYTZhNmM1YjI3YzdiODkwYjllMzU0MzJl";
}

//Functions to make http requests
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp){
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

std::string astronomyapirequest(std::string URL, std::string AuthString)
{
    // Fetch data
    CURL * curl;
    struct curl_slist *headerlist = NULL;    
    CURLcode res;
    std::string readBuffer;
    headerlist = curl_slist_append(headerlist, AuthString.c_str());
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, URL.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }
    return readBuffer;
}

//====================AstronomyAPI functions=======================//
//Bodies

std::string getBodyPosition(std::string body, ObserverParams Observer, std::string AuthString)
{
    std::string URL = "https://api.astronomyapi.com/api/v2/bodies/positions/" + body + buildQueryString(Observer);
    return astronomyapirequest(URL,AuthString);
}


//Studio


//Search




