#include <iostream>
#include <string>
#include <curl/curl.h>





std::string astronomyapirequest(std::string URL)
{
    std::string HEADER = "Authorization: Basic YzJhNTM2NzktNTQzYS00MDBmLThmMDMtNjI1ZTc2MmFhYjkxOjE5NWQ4ZjNhYjFmM2M1Y2YyNjdiODEwY2IzNDM2OGExMWExNmUwNDNlY2Q1ZWRiOGJmNDVlMzZiOGMxOTliMGFhMjU0NzFhNTZmOTc1YzBlNzFjNTYwZmJlYmY1ZjdkNjJhM2JkZTQzMTY3NDhhMjhiMWNlNTljYTExNzUwYWRkOWExZWI3YzVhOGU3NDM3ZGVmOGM1YWQ1ZTVlYzQ5ZGQ0ZWNjZDg3YTg4OTIzOGQ1YjA0ZWI0ZGFhMDIxMGQzNjdhZjMwZmVhYTZhNmM1YjI3YzdiODkwYjllMzU0MzJl";
    // Fetch data
    CURL * curl;
    struct curl_slist *headerlist = NULL;    
    CURLcode res;
    std::string readBuffer;
    headerlist = curl_slist_append(headerlist, HEADER.c_str());
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


std::string getPosition(std::string Target, std::string Location)
{
    std::string URLstring = "https://api.astronomyapi.com/api/v2/bodies/positions/" + Target + "?longitude=-84.39733&latitude=33.775867&elevation=1&from_date=2023-05-04&to_date=2023-05-04&time=13%3A57%3A57";
    return astronomyapirequest(URLstring);
}

std::string generateLocationString(double Lattitude, double Longitude, double Elevation, std::string StartDate, std::string EndDate, std::string Time)
{
    return "longitude=" + to_string(Longitude) + "&lattitude=" + to_string(Lattitude) + "&elevation=" + to_string(Elevation) + "&from_date=" + to_string(StartDate) + "&to_date=" + to_string(EndDate) + "&time=" + to_string(Time);

}


int main()
{
    //Define Targets
    std::string target = "jupiter";
    std::string location = "temp";
    std::string position_string = getPosition(target,location);

    std::cout << position_string << std::endl;


    return 0;
}