#include <iostream>
#include <string>
#include <curl/curl.h>

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp){
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}


std::string BuildQueryString(std::string targetbody)
{
    return "https://api.astronomyapi.com/api/v2/bodies/positions/" + targetbody + "?longitude=-84.39733&latitude=33.775867&elevation=1&from_date=2023-05-04&to_date=2023-05-04&time=13%3A57%3A57";
}

int main()
{
    //Define Targets
    std::string Target = "jupiter";

    std::string URLstring = BuildQueryString(Target);
    std::cout << URLstring << std::endl;
    URLstring = "https://api.astronomyapi.com/api/v2/bodies/positions/jupiter?longitude=-84.39733&latitude=33.775867&elevation=1&from_date=2023-05-04&to_date=2023-05-04&time=13%3A57%3A57";






    // Fetch data
    CURL * curl;
    struct curl_slist *headerlist = NULL;
    
    CURLcode res;
    std::string readBuffer;

    headerlist = curl_slist_append(headerlist, "Authorization: Basic YzJhNTM2NzktNTQzYS00MDBmLThmMDMtNjI1ZTc2MmFhYjkxOjE5NWQ4ZjNhYjFmM2M1Y2YyNjdiODEwY2IzNDM2OGExMWExNmUwNDNlY2Q1ZWRiOGJmNDVlMzZiOGMxOTliMGFhMjU0NzFhNTZmOTc1YzBlNzFjNTYwZmJlYmY1ZjdkNjJhM2JkZTQzMTY3NDhhMjhiMWNlNTljYTExNzUwYWRkOWExZWI3YzVhOGU3NDM3ZGVmOGM1YWQ1ZTVlYzQ5ZGQ0ZWNjZDg3YTg4OTIzOGQ1YjA0ZWI0ZGFhMDIxMGQzNjdhZjMwZmVhYTZhNmM1YjI3YzdiODkwYjllMzU0MzJl");
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, URLstring.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        std::cout << readBuffer << std::endl;
    }

    return 0;

}