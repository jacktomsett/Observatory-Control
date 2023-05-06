#include <iostream>
#include <string>
#include <curl/curl.h>

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp){
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

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



int main()
{
    //Define Targets
    std::string target = "jupiter";
    std::string location = "temp";
    std::string position_string = getPosition(target,location);

    std::cout << position_string << std::endl;


    return 0;
}