#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include "Classes.cpp"
const double pi = 3.14159265358979323846;
const int SecondsInDay = 86400;
//=====================================================================//
//                           Functions                                 //
//=====================================================================//



std::time_t createDateTime(int year, int month, int day, int hour, int minute, int second)
{
    std::tm temp = tm();
    temp.tm_year = year - 1900;
    temp.tm_mon = month -1;
    temp.tm_mday = day;
    temp.tm_hour = hour;
    temp.tm_min = minute;
    temp.tm_sec = second;
    return mktime(&temp);
}

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


double * calculateAltAz(double lattitude, double longitude, double RightAccension, double Declination, std::time_t ObservationDateTime)
{    
    static double AltAz[2];   //Cannot return an array in c++. Instead declare a static array and return a pointer to it

    //Generate a time_t object for celestial epoch:
    std::time_t epoch = createDateTime(2000,1,1,0,0,0);
    //Calculate number of days between then an observation date-time:
    double Ndays = difftime(ObservationDateTime,epoch)/86400;
    //extract decimal time from observation date time
    struct tm * Obs;
    Obs = gmtime(&ObservationDateTime);


    double TimeOfDay = Obs->tm_hour + (Obs->tm_min / 60.0) ;
    std::cout << "Time of day: " << TimeOfDay << std::endl;

    double RightAccensionDeg = RightAccension * 15;    


    double LocalMeanSiderealTime = (100.46 + (0.985647 * Ndays) + longitude + (15 * TimeOfDay));

    if (LocalMeanSiderealTime > 360.0)
    {
        while (LocalMeanSiderealTime > 360)
        {
            LocalMeanSiderealTime = LocalMeanSiderealTime - 360.0;
        }
    }
    else
    {
        while (LocalMeanSiderealTime < 0.0)
        {
            LocalMeanSiderealTime = LocalMeanSiderealTime + 360.0;
        }
    }

//    std::cout << "LocalMeanSideralTime:" << LocalMeanSiderealTime << std::endl;
    double HourAngle = LocalMeanSiderealTime - RightAccensionDeg;
    if(HourAngle < 0)
    {
        HourAngle = HourAngle + 360;
    }
//    std::cout << "HourAngle:           " << HourAngle << std::endl;
    double sinalt = (sin(Declination*pi/180.0)*sin(lattitude*pi/180.0)) + (cos(Declination*pi/180.0)*cos(lattitude*pi/180.0)*cos(HourAngle*pi/180.0));
//    std::cout << "sinalt = " << sin(Declination*(pi/180.0)) << " * " << sin(lattitude*(pi/180.0)) << " + " << cos(Declination*(pi/180.0)) << " * " << cos(lattitude*(pi/180.0)) << " * " << cos(HourAngle*(pi/180.0)) << std::endl;
//    std::cout << "sinalt:             " << sinalt << std::endl;
    AltAz[0] = asin(sinalt)*180.0/pi;
    double cosaz = (sin(Declination*pi/180.0)-(sin(AltAz[0]*pi/180.0)*sin(lattitude*pi/180.0)))/(cos(AltAz[0]*pi/180.0)*cos(lattitude*pi/180.0));
//    std::cout << "cosA:               " << cosaz << std::endl;
    if (sin(HourAngle*pi/180.0) < 0)
    {
        AltAz[1] = acos(cosaz) * (180/pi);
    }
    else
    {
        AltAz[1] = ((2 * pi) - acos(cosaz))*(180/pi);
    }

    return AltAz;
}



//============================================================================//
//                                   main                                     //
//============================================================================//
int main()
{

    //Observer and target details:

    double ObserverLattDeg = 52;
    double ObserverLattMins = 30;
    double ObserverLongDeg = -2;
    double ObserverLongMins = 45;
    double ObserverLongDecimalDegrees = ObserverLongDeg + (ObserverLongMins / 60.0);
    double ObserverLattDecimalDegrees = ObserverLattDeg + (ObserverLattMins / 60.0);
    std::cout << "Observer Lattitude: " << ObserverLattDeg << " degrees, " << ObserverLattMins << " minutes." << std::endl;
    std::cout << "Observer Longitude: " << ObserverLongDeg << " degrees, " << ObserverLongMins << " minutes." << std::endl;


    CelestialObject Target = CelestialObject("Messier",45);
    std::cout << "Selected target: " << Target.getName() << "." << std::endl;
    std::cout << "RA: " << Target.getRA() << "h " << std::fmod(Target.getRA(),1.0)*60.0 << "m, Dec: " << Target.getDec() << "° " << std::fmod(Target.getDec(),1.0)*60.0 <<  " m." << std::endl;


    //Enter observation date and time:
    int ObservationYear = 2022;
    int ObservationMonth = 11;
    int ObservationDay = 2;
    int ObservationHour = 3;
    int ObservationMin = 15;
    int ObservationSec = 0;

    //import data into a ctime object:
    std::time_t ObservationTimePoint = createDateTime(ObservationYear,ObservationMonth,ObservationDay,ObservationHour,ObservationMin,ObservationSec);
    std::cout << "Observation time:" << std::endl;
    std::cout << ctime(&ObservationTimePoint) << std::endl;
    
    
    //Calculate position in sky
    double *testAltAz = calculateAltAz(ObserverLattDecimalDegrees,ObserverLongDecimalDegrees,Target.getRA(),Target.getDec(),ObservationTimePoint);
    std::cout << "Altitude: " << *testAltAz << std::endl;
    std::cout << "Azimuth : " << *(testAltAz+1) << std::endl;

    //Settings for Nikon D3500
    //Sensor is 23.5 x 15.6 mm
    //Image size is 6000 x 4000 px
    double px = 23.5/6000;
    double py = 15.6/4000;
    double p = (px + py)/2;
    std::cout << "Pixel pitch: " << p << " mm" << std::endl;

    //Need to check my understanding of the relationship between aperture, focal length and f number
    //Also need to look into how to choose a focal length/aperture.
    double aperture = 4;
    std::cout << "Aperture: " << aperture << std::endl;
    double fnumber = 4;
    double focallength = fnumber * aperture;
    std::cout << "Focal length: " << focallength << std::endl;


    std::cout << "Exposure: " << std::endl;
    std::cout << "Simplified NPF rule: " << simpleNPF(aperture,p,focallength) << "s" << std::endl;
    std::cout << "Full NPF rule:       " << fullNPF(aperture,p,focallength,Target.getDec()*pi/180) << "s" << std::endl<<std::endl;



    return 0;
};





