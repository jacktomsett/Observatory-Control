#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
const double pi = 3.618;
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


double * calculateAltAz(double lattitude, double longitude, double RightAccension, double Declination, double Ndays, double Time)
{
    
    static double AltAz[2];   //Cannot return an array in c++. Instead declare a static array and return a pointer to it

    std::cout << "Lattitude:          " << lattitude << std::endl;
    std::cout << "Longitude:          " << longitude << std::endl;
    std::cout << "Right Accension:    " << RightAccension << std::endl;
    std::cout << "Declination:        " << Declination << std::endl;
    std::cout << "Days since J2000:   " << Ndays << std::endl;
    std::cout << "Time:               " << Time << " hrs" << std::endl;
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

    std::cout << "LocalMeanSideralTime:" << LocalMeanSiderealTime << std::endl;
    double HourAngle = LocalMeanSiderealTime - RightAccension;
    if(HourAngle < 0)
    {
        HourAngle = HourAngle + 360;
    }
    std::cout << "HourAngle:           " << HourAngle << std::endl;
    double sinalt = (sin(Declination*pi/180.0)*sin(lattitude*pi/180.0)) + (cos(Declination*pi/180.0)*cos(lattitude*pi/180.0)*cos(HourAngle*pi/180.0));
    std::cout << "sinalt:             " << sinalt << std::endl;
    AltAz[0] = asin(sinalt)*180.0/pi;
    double cosaz = (sin(Declination*pi/180.0)-(sin(AltAz[0]*pi/180.0)*sin(lattitude*pi/180.0)))/(cos(AltAz[0]*pi/180.0)*cos(lattitude*pi/180.0));
    std::cout << "cosA:               " << cosaz << std::endl;
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
    double ObserverLatt = 1;
    double ObserverLong = 1;

    double TargetRA = 1;
    double TargetDec = 1;

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
    time_t epoch_t = mktime(epoch);

    //Display the contents of both time structures:
    std::cout << "Celestial epoch:" << std::endl;
    std::cout << "Year: "<< 1900 + epoch->tm_year << std::endl;
    std::cout << "Month: "<< 1 + epoch->tm_mon<< std::endl;
    std::cout << "Day: "<< epoch->tm_mday << std::endl;
    std::cout << "Time: "<< 1 + epoch->tm_hour << ":";
    std::cout << 1 + epoch->tm_min << ":";
    std::cout << 1 + epoch->tm_sec << std::endl<<std::endl;


    //Enter date to calculate position for
    struct tm * ObservationTime;
    int d_sec = 0;
    int d_min = 30;
    int d_hour = 23;
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
    time_t Observation_t = mktime(ObservationTime);

    


    std::cout << "Observation Time:" << std::endl;
    std::cout << "Year: "<< 1900 + ObservationTime->tm_year << std::endl;
    std::cout << "Month: "<< 1 + ObservationTime->tm_mon<< std::endl;
    std::cout << "Day: "<< ObservationTime->tm_mday << std::endl;
    std::cout << "Time: "<< 1 + ObservationTime->tm_hour << ":";
    std::cout << 1 + ObservationTime->tm_min << ":";
    std::cout << 1 + ObservationTime->tm_sec << std::endl<<std::endl;

    double NdaysSinceEpoch = difftime(Observation_t,epoch_t)/SecondsInDay;
    double ObsTimeDecimalHours = ObservationTime->tm_hour + ((ObservationTime->tm_min)/60.0);
    std::cout << "Number of days since epoch: " << NdaysSinceEpoch << " days" << std::endl;
    std::cout << "Decimal Hours:              " << ObsTimeDecimalHours << std::endl;

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
    

    double *testAltAz = calculateAltAz(ObserverLatt,ObserverLong,TargetRA,TargetDec,NdaysSinceEpoch,ObsTimeDecimalHours);
    std::cout << "Altitude: " << *testAltAz << std::endl;
    std::cout << "Azimuth : " << *(testAltAz+1) << std::endl;





    return 0;
};





