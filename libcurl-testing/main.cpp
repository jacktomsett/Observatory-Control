#include <iostream>
#include <string>
#include <curl/curl.h>
#include <astronomyapi_interface.cpp>


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
    Whiston.EndDate = Date(2023,05,11);
    Whiston.time = "21:30:00";

    std::string target = "jupiter";
    std::string position_string = getBodyPosition(target,Whiston,generateAuthString(AppID,AppSecretKey));

    std::cout << position_string << std::endl;


    return 0;
}