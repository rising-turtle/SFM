/*
	
	July 8, 2020, He Zhang, hzhang8@vcu.edu 


	run Structure From Motion, in order to test the idea of multi-frame error model 

*/


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>

#include "sfm.h"

using namespace std ; 

string sData_path("../../data"); 

vector<vector<feat_obs> > read_data(int N = 100); 


int main(int argc, char* argv[])
{
	int N = 10; 
 	vector<vector<feat_obs> > all_obs = read_data(N); 

 	// do_it 
 	GlobalSFM sfm; 
 	sfm.do_it(N, all_obs); 

	return 0; 
}



vector<std::vector<feat_obs> > read_data(int N)
{
	// string sImage_file = sConfig_path + "MH_05_cam0.txt";
	string sImage_file = sData_path + "/Features/";
	string sTimestamp_file = sImage_file + "/Timestamp.txt";

	ifstream fsTimestamp; //modified
	fsTimestamp.open(sTimestamp_file.c_str());//Added

	if (!fsTimestamp.is_open()){
		cerr << "Failed to open Timestamp file! " << sTimestamp_file << endl;
		return;
	}

	// random noise generator 
	std::random_device rd;
    std::default_random_engine generator_(rd());

    double g_sigma_pix = 0; 

    vector<std::vector<feat_obs> > all_obs(N); 

	for(int i=0; i<N; i++){

		std::string sImage_line;
		double dStampNSec;
		double dStampNSec_pre = 0;
		string sImgFileName;
		if(std::getline(fsTimestamp, sImage_line) && !sImage_line.empty()){
			std::istringstream ssImuData(sImage_line);
			ssImuData >> dStampNSec >> sImgFileName;
			
			string imagePath = sData_path + "/" +  sImgFileName;
			
			ifstream fsFeatures;
	 		fsFeatures.open(imagePath.c_str());
			
			if(!fsFeatures.is_open()){
				cerr << "Failed to open TimeStep file! " << sTimestamp_file << endl;
				return;
			}

			// std::vector<Vector2d> feature_positions_un;
			// std::vector<Vector3d> feature_positions; 
			// std::vector<Vector3d> feature_positions_pre;

			std::string sFeature_line;
			vector<feat_obs> v_feat_obs; 

			while (std::getline(fsFeatures, sFeature_line) && !sFeature_line.empty())
			{

				std::istringstream ssFeatureData(sFeature_line);
				// ssFeatureData >> landmark.x() >> landmark.y() >> landmark.z() >> landmark.w() >> feature_position_un.x() >> feature_position_un.y() >> feature_depth;

				feat_obs obs; 

				double w;  

				ssFeatureData >> obs.xyz[0] >> obs.xyz[1] >> obs.xyz[2] >> w >> obs.uv[0] >> obs.uv[1] >> obs.depth ;  

				// add noise 
				// if(g_sigma_pix > 0){
	    			// static std::normal_distribution<double> noise(0.0, g_sigma_pix/FOCAL_LENGTH);
					// feature_position_un(0) = feature_position_un(0) + noise(generator_); 
					// feature_position_un(1) = feature_position_un(1) + noise(generator_);
				// }

				v_feat_obs.push_back(obs); 

				//Record
				//landmarks.push_back(landmark); //Pixel coordinate
				// feature_positions_un.push_back(feature_position_un);
				// feature_position.x() = feature_position_un.x()*fx + cx;
				// feature_position.y() = feature_position_un.y()*fy + cy;
				// feature_position.z() = feature_depth; 
				// if (feature_position.x()>0&&feature_position.x()<640&&feature_position.y()>0&&feature_position.y()<480){
				
				// feature_positions.push_back(feature_position);

				// count++;
			}

			fsFeatures.close();

			all_obs[i] = v_feat_obs; 

		}else{
			cout<<"main_sfm.cpp: failed to getline "<<sImage_line<<endl; 
			break; 
		}
	}

	return all_obs; 
}