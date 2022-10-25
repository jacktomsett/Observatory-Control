#include <iostream>


double fullNPF(double Aperture, double pixelsize, double FocalLength, double declination)
{
    double Exposure = ((0.0997 *  FocalLength) + (16.856 * Aperture) + (13.713 * pixelsize)) / (FocalLength * cos(declination));
    return Exposure
}
double simpleNPF(double Aperture, double PixelSize, double FocalLength)
{
    double Exposure = ((35 * Aperture) + (30 * PixelSize))/FocalLength;
    return Exposure
}

int main()
{
    //Settings for Nikon D3500
    //Sensor is 23.5 x 15.6 mm
    //Image size is 6000 x 4000 px
    double px = 23.5/6000;
    double py = 15.6/4000;

    double p = (px + py)/2;


    double aperture = 4;
    double fnumber = 4;
    double focallength = fnumber * aperture;

    double Dec = 41;

    std::cout << "Exposure: " << std::endl;
    std::cout << "Simplified NPF rule: " << simpleNPF(aperture,p,focallength) << "s" << std::endl;
    std::cout << "Full NPF rule:       " << fullNPF(Aperture,p,focallength,declination) << "s" << std::endl;

    return 0;
};
