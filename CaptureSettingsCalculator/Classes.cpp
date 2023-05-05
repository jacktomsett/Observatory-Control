#ifndef _CLASSES_
#define _CLASSES_

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
        double getDec();
        int getNumber();
        std::string getName();
        std::string getType();
        double getMagnitude();
        std::string getConstellation();
        double getDistance();
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

        Magnitude = stod(MAGstring);
        //Distance = stod(DISTstring);

        std::stringstream RAstream(RAstring);
        std::string RAhourstring;
        std::string RAminstring;
        getline(RAstream,RAhourstring,' ');
        getline(RAstream,RAminstring,' ');
        //remove the units from the end of the string
        RAhourstring = RAhourstring.substr(0,RAhourstring.size()-1);
        RAminstring = RAminstring.substr(0,RAminstring.size()-1);

        std::stringstream Decstream(Decstring);
        std::string DecDegstring;
        std::string Decminstring;
        char delim = '°';
        getline(Decstream,DecDegstring,delim);
        getline(Decstream,Decminstring,delim);
        //remove the units from the end of the string
        DecDegstring = DecDegstring.substr(0,DecDegstring.size()-1);
        Decminstring = Decminstring.substr(0,Decminstring.size()-1);

        //convert to decimal
        RightAscension = stod(RAhourstring) + (stod(RAminstring)/60.0);
        Declination = stod(DecDegstring) + (stod(Decminstring)/60.0);

        catalogfile.close();
    }
    else
    {
        std::cout << "ERROR: unrecognised catalog" << std::endl;
    }

}

double CelestialObject::getRA()
{
    return RightAscension;
}
double CelestialObject::getDec()
{
    return Declination;
}
int CelestialObject::getNumber()
{
    return CatalogNumber;
}
std::string CelestialObject::getName()
{
    return Name;
}
std::string CelestialObject::getType()
{
    return Type;
}
double CelestialObject::getMagnitude()
{
    return Magnitude;
}
std::string CelestialObject::getConstellation()
{
    return Constellation;
}
double CelestialObject::getDistance()
{
    return Distance;
}

#endif // _CLASSES_
