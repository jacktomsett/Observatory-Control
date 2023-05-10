#include <string>
#include <curl/curl.h>

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


//=========To Do.....
std::string generateAuthString(std::string ApplicationID, std::string ApplicationSecret)
{

    return "0"
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

//AstronomyAPI functions
