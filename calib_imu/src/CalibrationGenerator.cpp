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

    calState=st_IDLE;

    proceed = false;
}

//------------------------------------------------------------------------------------//

CalibrationGenerator::~CalibrationGenerator() {
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::CalibrationStep(arma::vec const & input_mag,arma::vec const & input_acc,arma::vec const & input_gyr) {

    char c;
    bool keyPressed=false;
    SolutionEntry accE;
    SolutionEntry magE;
    SolutionEntry gyrE;

    keyPressed=proceed;
    proceed=false;

    switch(calState) {
    case st_IDLE:
        if(keyPressed) {

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
        if(keyPressed) {
            //puts("Align Gravitation to negative z direction.");
            calState=st_PAUSE1;
        }
        break;
    case st_PAUSE1:
        if(keyPressed) {

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
        if(keyPressed) {
            //puts("Align Gravitation to positive x direction.");
            calState=st_PAUSE2;
        }
        break;
    case st_PAUSE2:
        if(keyPressed) {

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
        if(keyPressed) {
            //puts("Align Gravitation to negative x direction.");
            calState=st_PAUSE3;
        }
        break;
    case st_PAUSE3:
        if(keyPressed) {

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
        if(keyPressed) {
            //puts("Align Gravitation to positive y direction.");
            calState=st_PAUSE4;
        }
        break;
    case st_PAUSE4:
        if(keyPressed) {

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
        if(keyPressed) {
            //puts("Align Gravitation to negative y direction.");
            calState=st_PAUSE5;
        }
        break;
    case st_PAUSE5:
        if(keyPressed) {

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
        if(keyPressed) {
            //puts("Turn the robot in a sphere.");
            calState=st_SPERE;
        }
        break;
    case st_SPERE:
        magE.Measured=input_mag;
        magE.Estimated=input_mag;
        // Scale vector
        magE.Estimated=magE.Estimated*(mag.norm_amplitude/(abs_vec(magE.Estimated)));
        mag_data.push_back(magE);
        if(keyPressed) {
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
            for(size_t ind=0; ind<3; ind++) {
                vec b(mag_data.size());
                mat C(mag_data.size(), 4);

                for(u32 i=0; i<mag_data.size(); ++i) {
                    b(i)   = mag_data[i].Estimated(ind);

                    C(i,0) = -1;
                    C(i,1) = mag_data[i].Measured(0);
                    C(i,2) = mag_data[i].Measured(1);
                    C(i,3) = mag_data[i].Measured(2);
                }

                vec solution = solve(C,b);

                mag.bias(ind)=solution(0);
                mag.combined(ind,0)=solution(1);
                mag.combined(ind,1)=solution(2);
                mag.combined(ind,2)=solution(3);
            }
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

            CalculateVectorsFromCombinedMatrix(gyr);
            CalculateVectorsFromCombinedMatrix(acc);
            CalculateVectorsFromCombinedMatrix(mag);


            calState=st_FINISHED;
        }
        break;
    case st_FINISHED:
        if(keyPressed) {
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

void CalibrationGenerator::CalculateVectorsFromCombinedMatrix(CalData & cal) {

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

