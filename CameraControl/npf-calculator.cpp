#include <iostream>
#include <fstream>
#include <math.h>
#include <jsoncpp/json/json.h>

int main(int argc, char *argv[]){
	
	//Print usage
	if (argc != 6){
		std::cout << "Usage: " << argv[0] << " FocalLength f-number Declination(degrees) CameraInfoFile.json MultiplicationFactor" << std::endl;
		return 1;
	}

	//Parse command line arguments
	double focal_length	= atof(argv[1]);
	double fnumber		= atof(argv[2]);
	double declination	= atof(argv[3]);
	double kfactor		= atof(argv[5]);
	//Calculate aperture
	double aperture = focal_length / fnumber;

	//Read in pixel pitch
	std::ifstream camerafile;
	camerafile.open(argv[4],std::ios::in);
	if(camerafile.is_open() != true){
		std::cerr << "ERROR: Cant open Camera Info file" << std::endl;
		return 1;
	}
	Json::Value cameradata;
	camerafile >> cameradata;
	camerafile.close();
	double pixelpitch = cameradata["pixel pitch"].asDouble();
	//std::cout << "Focal Length : " << focal_length << std::endl;
	//std::cout << "f-number     : " << fnumber << std::endl;
	//std::cout << "Aperture     : " << aperture << std::endl;
	//std::cout << "Pixel Pitch  : " << pixelpitch << std::endl;

	double simple_exposure = ((35 * fnumber) + (30 * pixelpitch))/focal_length;
	//std::cout << "Simple NPF   : " << simple_exposure << std::endl;
	double complex_exposure = kfactor * ( (16.856 * fnumber) + (0.0997*focal_length) + (13.713*pixelpitch) )/ (focal_length * cos(declination * 3.14159265/180) );
	//std::cout << "Complex NPF  : " << complex_exposure << std::endl;
	std::cout << complex_exposure << std::endl;
	return 0;

}
