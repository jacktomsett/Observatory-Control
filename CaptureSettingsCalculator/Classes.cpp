#include <string>
#include <iostream>
#include <fstream>
#include <sstream>



class CelestialObject
{
    private:
        std::string Catalog;
        double RightAscension;
        double Declination;
        int CatalogNumber;
        std::string Name;
        std::string Type;
        double Magnitude;
        std::string Constellation;
        double Distance;

    public:
        CelestialObject(std::string,int);
        CelestialObject(std::string);
        double getRA();
        double getdec();
        int getNumber();
        std::string getName();
        std::string getType();
};

CelestialObject::CelestialObject(std::string catalog, int number)
{
    if (catalog == "Messier")
    {
        Catalog = "Messier";
        CatalogNumber = number;

        //open file:
        std::string filename = "/home/jack/Coding/Observatory/CaptureSettingsCalculator/Messier_Catalog.csv";
        std::ifstream catalogfile;
        catalogfile.open(filename);
        if (catalogfile.is_open() == false)
        {
            std::cout << "ERROR: failed to open catalog file" << std::endl;
        }


        //extract entry from file
        std::string line;
        int i = 1;
        while (i <= number+1)
        {
            getline(catalogfile,line);
            i++;
        }

        //Extract data from entry
        std::string RAstring;
        std::string Decstring;
        std::string MAGstring;
        std::string DISTstring;

        std::stringstream linestream(line);
        std::string junkstring;

        getline(linestream,junkstring,',');
        getline(linestream,Name,',');
        getline(linestream,Type,',');
        getline(linestream,Constellation,',');
        getline(linestream,RAstring,',');
        getline(linestream,Decstring,',');
        getline(linestream,MAGstring,',');
        getline(linestream,junkstring,',');
        getline(linestream,DISTstring,',');

        std::cout << "Selected target is " <<  Name << ", a " << Type << " in the constellation " << Constellation << ". It has a Right accension of " << RAstring << " and a declination of " << Decstring << ". " << std::endl;

        Magnitude = stod(MAGstring);

        catalogfile.close();
    }
    else
    {
        std::cout << "ERROR: unrecognised catalog" << std::endl;
    }

}
