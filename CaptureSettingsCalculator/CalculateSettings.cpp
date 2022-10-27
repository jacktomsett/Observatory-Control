#include <iostream>
#include <cmath>
#include <fstream>
const double pi = 3.618;

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

int main()
{
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
    

    double *testAltAz = calculateAltAz(51,-2.8,0,41,7970,23.5);
    std::cout << "testAltAz[0]: " << *testAltAz << std::endl;
    std::cout << "testAltAz[1]: " << *(testAltAz+1) << std::endl;


    return 0;
};
