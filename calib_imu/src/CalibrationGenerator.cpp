/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     CalibrationGenerator.cpp
 * \license  BSD-3-License
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <iostream>

#include "CalibrationGenerator.h"

using namespace std;
using namespace arma;

/**************************************************************************************
 * MACROS
 **************************************************************************************/
//Macro used to calculate the absolute value of a vector
#define abs_vec(_vec) (sqrt((_vec)(0)*(_vec)(0)+(_vec)(1)*(_vec)(1)+(_vec)(2)*(_vec)(2)))

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

/**************************************************************************************
 * PUBLIC FUNCTIONS
 **************************************************************************************/

CalibrationGenerator::CalibrationGenerator(double norm_amplitude_mag,double norm_amplitude_acc, double norm_amplitude_gyr) {
    mag.norm_amplitude=norm_amplitude_mag;
    acc.norm_amplitude=norm_amplitude_acc;
    gyr.norm_amplitude=norm_amplitude_gyr;

	//Initialize calibration matrices with default values 
	//Calibration with this values does the same as no calibration at all.
    mag.combined=mat(3,3,fill::eye);
    mag.bias=vec(3,fill::zeros);
    mag.sensitivity=vec(3,fill::zeros);
    mag.orthogonalisation=vec(3,fill::zeros);
    mag.alignement=vec(3,fill::zeros);

    acc.combined=mat(3,3,fill::eye);
    acc.bias=vec(3,fill::zeros);
    acc.sensitivity=vec(3,fill::zeros);
    acc.orthogonalisation=vec(3,fill::zeros);
    acc.alignement=vec(3,fill::zeros);

    gyr.combined=mat(3,3,fill::eye);
    gyr.bias=vec(3,fill::zeros);
    gyr.sensitivity=vec(3,fill::zeros);
    gyr.orthogonalisation=vec(3,fill::zeros);
    gyr.alignement=vec(3,fill::zeros);

	//The statemachine starts with the idle state set
    calState=st_IDLE;
	//First command for the user
	puts("Align Gravitation to positive z direction.");

	//No user input till now
    proceed = false;
}

//------------------------------------------------------------------------------------//

CalibrationGenerator::~CalibrationGenerator() {
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::CalibrationStep(arma::vec const & input_mag,arma::vec const & input_acc,arma::vec const & input_gyr) {


	SolutionEntry accE;
    SolutionEntry magE;
    SolutionEntry gyrE;
    bool advanceStatemachine=false;
	
	//If a user input has occured since the last step - advance the statemachine
    advanceStatemachine=proceed;
    proceed=false;
	
	//The statemachine stores the measured values alongside the expected values according to the current state.
	//After the final state a matrice coefficient is determined per sensor - which should convert measured values to the best possible estimation.

    switch(calState) {
    case st_IDLE:
        if(advanceStatemachine) {

            calState=st_ZP;
        }
        break;
    case st_ZP:
        //Accelerometer
        accE.Measured=input_acc;
        accE.Estimated=vec(3,fill::zeros);
        accE.Estimated(0)=0.0;
        accE.Estimated(1)=0.0;
        accE.Estimated(2)=acc.norm_amplitude;
        gyrE.Measured=input_gyr;
        gyrE.Estimated=vec(3,fill::zeros);
        gyr_data.push_back(gyrE);
        acc_data.push_back(accE);
        if(advanceStatemachine) {
            puts("Align Gravitation to negative z direction.");
            calState=st_PAUSE1;
        }
        break;
    case st_PAUSE1:
        if(advanceStatemachine) {

            calState=st_ZN;
        }
        break;
    case st_ZN:
        accE.Measured=input_acc;
        accE.Estimated=vec(3,fill::zeros);
        accE.Estimated(0)=0.0;
        accE.Estimated(1)=0.0;
        accE.Estimated(2)=-acc.norm_amplitude;
        gyrE.Measured=input_gyr;
        gyrE.Estimated=vec(3,fill::zeros);
        gyr_data.push_back(gyrE);
        acc_data.push_back(accE);
        if(advanceStatemachine) {
            puts("Align Gravitation to positive x direction.");
            calState=st_PAUSE2;
        }
        break;
    case st_PAUSE2:
        if(advanceStatemachine) {

            calState=st_XP;
        }
        break;
    case st_XP:
        accE.Measured=input_acc;
        accE.Estimated=vec(3,fill::zeros);
        accE.Estimated(0)=acc.norm_amplitude;
        accE.Estimated(1)=0.0;
        accE.Estimated(2)=0.0;
        gyrE.Measured=input_gyr;
        gyrE.Estimated=vec(3,fill::zeros);
        gyr_data.push_back(gyrE);
        acc_data.push_back(accE);
        if(advanceStatemachine) {
            puts("Align Gravitation to negative x direction.");
            calState=st_PAUSE3;
        }
        break;
    case st_PAUSE3:
        if(advanceStatemachine) {

            calState=st_XN;
        }
        break;
    case st_XN:
        accE.Measured=input_acc;
        accE.Estimated=vec(3,fill::zeros);
        accE.Estimated(0)=-acc.norm_amplitude;
        accE.Estimated(1)=0.0;
        accE.Estimated(2)=0.0;
        gyrE.Measured=input_gyr;
        gyrE.Estimated=vec(3,fill::zeros);
        gyr_data.push_back(gyrE);
        acc_data.push_back(accE);
        if(advanceStatemachine) {
            puts("Align Gravitation to positive y direction.");
            calState=st_PAUSE4;
        }
        break;
    case st_PAUSE4:
        if(advanceStatemachine) {

            calState=st_YP;
        }
        break;
    case st_YP:
        accE.Measured=input_acc;
        accE.Estimated=vec(3,fill::zeros);
        accE.Estimated(0)=0.0;
        accE.Estimated(1)=acc.norm_amplitude;
        accE.Estimated(2)=0.0;
        gyrE.Measured=input_gyr;
        gyrE.Estimated=vec(3,fill::zeros);
        gyr_data.push_back(gyrE);
        acc_data.push_back(accE);
        if(advanceStatemachine) {
            puts("Align Gravitation to negative y direction.");
            calState=st_PAUSE5;
        }
        break;
    case st_PAUSE5:
        if(advanceStatemachine) {

            calState=st_YN;
        }
        break;
    case st_YN:
        accE.Measured=input_acc;
        accE.Estimated=vec(3,fill::zeros);
        accE.Estimated(0)=0.0;
        accE.Estimated(1)=-acc.norm_amplitude;
        accE.Estimated(2)=0.0;
        gyrE.Measured=input_gyr;
        gyrE.Estimated=vec(3,fill::zeros);
        gyr_data.push_back(gyrE);
        acc_data.push_back(accE);
        if(advanceStatemachine) {
            puts("Turn the robot in a sphere.");
            calState=st_SPERE;
        }
        break;
    case st_SPERE:
        magE.Measured=input_mag;
        magE.Estimated=input_mag;
        // Stretch the ellipsoid to a sphere
        magE.Estimated=magE.Estimated*(mag.norm_amplitude/(abs_vec(magE.Estimated)));
        mag_data.push_back(magE);
		
        if(advanceStatemachine) {
			//Before finishing do the math to calculate calibration data from aquired data
			CalculateCombinedMatrixFromSolutionEntries();
            CalculateVectorsFromCombinedMatrix(gyr);
            CalculateVectorsFromCombinedMatrix(acc);
            CalculateVectorsFromCombinedMatrix(mag);

            calState=st_FINISHED;
        }
        break;
    case st_FINISHED:
        if(advanceStatemachine) {
            calState=st_FINISHED;
        }
        break;
    }
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::InitialiseCalibrationObjectMag(Sensor3DCalibration & cal) {
    cal.SetBiasValues(mag.bias);
    cal.SetSensitivityValues(mag.sensitivity);
    cal.SetOrthogonalisationValues(mag.orthogonalisation);
    cal.SetAlignementValues(mag.alignement);
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::InitialiseCalibrationObjectAcc(Sensor3DCalibration & cal) {
    cal.SetBiasValues(acc.bias);
    cal.SetSensitivityValues(acc.sensitivity);
    cal.SetOrthogonalisationValues(acc.orthogonalisation);
    cal.SetAlignementValues(acc.alignement);
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::InitialiseCalibrationObjectGyr(Sensor3DCalibration & cal) {
    cal.SetBiasValues(gyr.bias);
    cal.SetSensitivityValues(gyr.sensitivity);
    cal.SetOrthogonalisationValues(gyr.orthogonalisation);
    cal.SetAlignementValues(gyr.alignement);
}

//------------------------------------------------------------------------------------//

bool CalibrationGenerator::isFinished() {
    return calState==st_FINISHED;
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::doUserInput() {
    proceed = true;
}

/**************************************************************************************
 * PRIVATE FUNCTIONS
 **************************************************************************************/

void CalibrationGenerator::CalculateCombinedMatrixFromSolutionEntries() {
	
	//The accelerometer data is fit to match the gravitational acceleration in all six alignements.
	for(size_t ind=0; ind<3; ind++) {
		vec b(acc_data.size());
		mat C(acc_data.size(), 4);

		for(u32 i=0; i<acc_data.size(); ++i) {
			b(i)   = acc_data[i].Estimated(ind);

			C(i,0) = -1;
			C(i,1) = acc_data[i].Measured(0);
			C(i,2) = acc_data[i].Measured(1);
			C(i,3) = acc_data[i].Measured(2);
		}

		vec solution = solve(C,b);

		acc.bias(ind)=solution(0);
		acc.combined(ind,0)=solution(1);
		acc.combined(ind,1)=solution(2);
		acc.combined(ind,2)=solution(3);
	}
	
	//For the magnetometer the aquired ellipsoid is fit to a sphere 
	//Currently this is done by adjusting the amplitude of the mesured data to fit the unity sphere
	//TODO: Implement a "better" algorithmn to fit the sphere
	//DONE: Now Ellipsoid fitting method by Yury Petrov.
	vec b(mag_data.size());
	mat C(mag_data.size(), 9);

	for(u32 i=0; i<mag_data.size(); ++i) {
		b(i)   = 1;

		C(i,1) = mag_data[i].Measured(0)*mag_data[i].Measured(0);
		C(i,2) = mag_data[i].Measured(1)*mag_data[i].Measured(1);
		C(i,3) = mag_data[i].Measured(2)*mag_data[i].Measured(2);
		C(i,4) = mag_data[i].Measured(0)*mag_data[i].Measured(1)*2.0;
		C(i,5) = mag_data[i].Measured(0)*mag_data[i].Measured(2)*2.0;
		C(i,6) = mag_data[i].Measured(1)*mag_data[i].Measured(2)*2.0;
		C(i,7) = mag_data[i].Measured(0)*2.0;
		C(i,8) = mag_data[i].Measured(1)*2.0;
		C(i,9) = mag_data[i].Measured(2)*2.0;
	}

	vec solution = solve(C,b);
	
	//TODO: get mapping from ellipsoid to an sphere

	
	//For the gyroscope only bias values may be calculated from the aquired data
	//Combined matrice is set to not affect the measured values -> identity matrice
	for(size_t ind=0; ind<3; ind++) {
		vec b(gyr_data.size());
		mat C(gyr_data.size(), 4);

		for(u32 i=0; i<gyr_data.size(); ++i) {
			b(i)   = gyr_data[i].Estimated(ind);
			C(i,0) = -1;
		}

		vec solution = solve(C,b);

		gyr.bias(ind)=solution(0);
		gyr.combined(0,0)=1;
		gyr.combined(1,1)=1;
		gyr.combined(2,2)=1;
	}	
}
 
//------------------------------------------------------------------------------------//
 
void CalibrationGenerator::CalculateVectorsFromCombinedMatrix(CalData & cal) {

	//Do some math magic to solve the equation system (coefficient comparision):
	//Combined_Matrice*(MeasuredDataVector-(Combined_Matrice^-1)*CombinedBias) = EstimatedDataVector
	//(AlignementMatrice^-1)*(OrthogonalisationMatrice^-1)*(SensitivityMatrice^-1)*(MeasuredData-Bias)=EstimatedDataVector
	//For the elements of the matrices see (http://www.sciencedirect.com/science/article/pii/S0924424707003834)

    cal.bias=inv(cal.combined)*cal.bias;

    cal.alignement(2)=atan2(cal.combined(1,0),cal.combined(0,0));
    cal.alignement(1)=atan2(-cal.combined(2,0)*sin(cal.alignement(2)),cal.combined(1,0));
    cal.alignement(0)=atan2(((cal.combined(1,2)/cal.combined(2,2))-tan(cal.alignement(1)*sin(cal.alignement(2))))*cos(cal.alignement(1)),-cos(cal.alignement(2)));

    cal.sensitivity(0)=cos(cal.alignement(1))*cos(cal.alignement(2))/cal.combined(0,0);
    cal.sensitivity(1)=(cos(cal.alignement(0))*cos(cal.alignement(2))+sin(cal.alignement(0))*sin(cal.alignement(1))*sin(cal.alignement(2)))/cal.combined(2,1);
    cal.sensitivity(2)=cos(cal.alignement(0))*cos(cal.alignement(1))/cal.combined(2,2);

    //Internal orthogonalisation cannot be estimated by the used calibration process, so estimate it to be ideal (90 deg)
    cal.orthogonalisation(0)=M_PI/2.0;
    cal.orthogonalisation(1)=M_PI/2.0;
    cal.orthogonalisation(2)=M_PI/2.0;
}

