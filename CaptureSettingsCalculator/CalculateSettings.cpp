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


double * calculateAltAz(double lattitude, double longitude, double RightAccension, double Declination, double Ndays,double Time)
{
    
    static double AltAz[2];   //Cannot return an array in c++. Instead declare a static array and return a pointer to it

//    std::cout << "Lattitude:          " << lattitude << std::endl;
//    std::cout << "Longitude:          " << longitude << std::endl;
//    std::cout << "Right Accension:    " << RightAccension << std::endl;
//    std::cout << "Declination:        " << Declination << std::endl;
//    std::cout << "Days since J2000:   " << Ndays << std::endl;
//    std::cout << "Time:               " << Time << " hrs" << std::endl;
    double RightAccensionDeg = RightAccension * 15;    


    double LocalMeanSiderealTime = (100.46 + (0.985647 * Ndays) + longitude + (15 * Time));

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

    double ObserverLattDeg = 42;
    double ObserverLattMins = 21;
    double ObserverLongDeg = -71;
    double ObserverLongMins = 4;

    double TargetRAHours = 3;
    double TargetRAMins = 47;
    double TargetDecDeg = 24;
    double TargetDecMins = 7;

    std::cout << "Observer Lattitude: " << ObserverLattDeg << " degrees, " << ObserverLattMins << " minutes." << std::endl;
    std::cout << "Observer Longitude: " << ObserverLongDeg << " degrees, " << ObserverLongMins << " minutes." << std::endl;
    std::cout << "Target Right Ascension: " << TargetRAHours << " hours, " << TargetRAMins <<  " minutes." << std::endl;
    std::cout << "Target Declination: " << TargetDecDeg << " degrees, " << TargetDecMins << " minutes." << std::endl;


    //conver values to decimal equivilents:
    double ObserverLongDecimalDegrees = ObserverLongDeg + (ObserverLongMins / 60.0);
    double ObserverLattDecimalDegrees = ObserverLattDeg + (ObserverLattMins / 60.0);
    double TargetRADecimalHours = TargetRAHours + (TargetRAMins / 60.0);
    double TargetDecDecimalDegrees = TargetDecDeg + (TargetDecMins / 60.0);


    //Enter observation date and time:
    int ObservationYear = 2004;
    int ObservationMonth = 4;
    int ObservationDay = 6;
    int ObservationHour = 21;
    int ObservationMin = 0;
    int ObservationSec = 0;

    //import data into a ctime object:
    std::tm tm{};
    tm.tm_year = ObservationYear - 1900;
    tm.tm_mon = ObservationMonth - 1;
    tm.tm_mday = ObservationDay;
    tm.tm_hour = ObservationHour;
    tm.tm_min = ObservationMin;
    //ObservationDateTime.tm_isdst = 0;
    std::time_t ObservationDateTime_timet = std::mktime(&tm);
    //std::tm ObservationDateTime_tm = std::gmtime(&ObservationDateTime_timet)
    //std::cout << "Observation time: " << std::put_time(&ObservationDateTime_tm, "%c %Z") << '\n';




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
    std::cout << "Full NPF rule:       " << fullNPF(aperture,p,focallength,Dec*pi/180) << "s" << std::endl<<std::endl;
    

//    double *testAltAz = calculateAltAz(ObserverLattDecimalDegrees,ObserverLongDecimalDegrees,TargetRADecimalHours,TargetDecDecimalDegrees,NdaysSinceEpoch,ObsTimeDecimalHours);
//    std::cout << "Altitude: " << *testAltAz << std::endl;
//    std::cout << "Azimuth : " << *(testAltAz+1) << std::endl;

    //Parse catalog for for desired object:
    CelestialObject Target = CelestialObject("Messier",31);



    return 0;
};





