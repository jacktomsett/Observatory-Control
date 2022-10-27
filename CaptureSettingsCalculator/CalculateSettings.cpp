#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
const double pi = 3.618;
//=====================================================================//
//                           Functions                                 //
//=====================================================================//

double fullNPF(double Aperture, double pixelsize, double FocalLength, double declination)
{
    double Exposure = ((0.0997 *  FocalLength) + (16.856 * Aperture) + (13.713 * pixelsize*1000)) / (FocalLength * cos(declination));
    return Exposure;
}
double simpleNPF(double Aperture, double PixelSize, double FocalLength)
{
    double Exposure = ((35 * Aperture) + (30 * PixelSize*1000))/FocalLength;
    return Exposure;
}


double * calculateAltAz(double lattitude, double longitude, double RightAccension, double Declination, double Ndays, double Time)
{
    
    static double AltAz[2];   //Cannot return an array in c++. Instead declare a static array and return a pointer to it

   
    double MeanSiderealTime = 100.46 + (0.985647 * Ndays) + (longitude*pi/180) + (15 * Time);
    double LocalMeanSideralTime = MeanSiderealTime + longitude;
    double HourAngle = LocalMeanSideralTime - RightAccension;

    double sinalt = (sin(Declination)*sin(lattitude)) + (cos(Declination)*cos(lattitude)*cos(HourAngle));
    AltAz[0] = asin(sinalt);
    double cosaz = (sin(Declination)-(sin(AltAz[0])*sin(lattitude)))/(cos(AltAz[0])*cos(lattitude));
    if (sin(HourAngle) < 0)
    {
        AltAz[1] = acos(cosaz);
    }
    else
    {
        AltAz[1] = (2 * pi) - acos(cosaz);
    }

    return AltAz;
}



//============================================================================//
//                                   main                                     //
//============================================================================//
int main()
{

    //import celestial epoch
    time_t rawtime;
    struct tm * epoch;

    int epochsec = 0;
    int epochmin = 0;
    int epochhour = 0;
    int epochday = 1;
    int epochmonth = 1;    
    int epochyear = 2000;

    time ( &rawtime );
    epoch = gmtime( &rawtime );
    epoch->tm_year = epochyear - 1900;
    epoch->tm_mon = epochmonth - 1;
    epoch->tm_mday = epochday;
    epoch->tm_hour = epochhour;
    epoch->tm_min = epochmin;
    epoch->tm_sec = epochsec;
    mktime(epoch);


    //Enter date to calculate position for
    struct tm * ObservationTime;
    int d_sec = 0;
    int d_min = 30;
    int d_hour = 11;
    int d_day = 30;
    int d_month = 10;    
    int d_year = 2022;

    ObservationTime = gmtime( &rawtime );
    ObservationTime->tm_year = d_year - 1900;
    ObservationTime->tm_mon = d_month - 1;
    ObservationTime->tm_mday = d_day;
    ObservationTime->tm_hour = d_hour;
    ObservationTime->tm_min = d_min;
    ObservationTime->tm_sec = d_sec;
    mktime(ObservationTime);

    
    //Display the contents of both time structures:
    std::cout << "Celestial epoch:" << std::endl;
    std::cout << "Year: "<< 1900 + epoch->tm_year << std::endl;
    std::cout << "Month: "<< 1 + epoch->tm_mon<< std::endl;
    std::cout << "Day: "<< epoch->tm_mday << std::endl;
    std::cout << "Time: "<< 1 + epoch->tm_hour << ":";
    std::cout << 1 + epoch->tm_min << ":";
    std::cout << 1 + epoch->tm_sec << std::endl<<std::endl;

    std::cout << "Observation Time:" << std::endl;
    std::cout << "Year: "<< 1900 + ObservationTime->tm_year << std::endl;
    std::cout << "Month: "<< 1 + ObservationTime->tm_mon<< std::endl;
    std::cout << "Day: "<< ObservationTime->tm_mday << std::endl;
    std::cout << "Time: "<< 1 + ObservationTime->tm_hour << ":";
    std::cout << 1 + ObservationTime->tm_min << ":";
    std::cout << 1 + ObservationTime->tm_sec << std::endl<<std::endl;

    //Settings for Nikon D3500
    //Sensor is 23.5 x 15.6 mm
    //Image size is 6000 x 4000 px
    double px = 23.5/6000;
    double py = 15.6/4000;
    double p = (px + py)/2;

    //Need to check my understanding of the relationship between aperture, focal length and f number
    //Also need to look into how to choose a focal length/aperture.
    double aperture = 4;
    double fnumber = 4;
    double focallength = fnumber * aperture;
    
    //Need to look into this also. Obviously the height above the horizon will change throughout the height. But also declination is a fixed coordinate. So maybe that is not the declination they are reffering to.
    double Dec = 41;

    std::cout << "Exposure: " << std::endl;
    std::cout << "Simplified NPF rule: " << simpleNPF(aperture,p,focallength) << "s" << std::endl;
    std::cout << "Full NPF rule:       " << fullNPF(aperture,p,focallength,Dec*pi/180) << "s" << std::endl;
    

    //double *testAltAz = calculateAltAz(51,-2.8,0,41,difftime(now,mktime(epoch))/86400,23.5);
    //std::cout << "testAltAz[0]: " << *testAltAz << std::endl;
    //std::cout << "testAltAz[1]: " << *(testAltAz+1) << std::endl;





    return 0;
};





