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

static void help_buildEquationSystem(arma::vec & b, arma::mat &C, size_t & i, arma::vec input_vector, arma::vec output_vector);

/**************************************************************************************
 * PUBLIC FUNCTIONS
 **************************************************************************************/

CalibrationGenerator::CalibrationGenerator(double norm_amplitude_mag,double norm_amplitude_acc, double norm_amplitude_gyr) {
    mag.norm_amplitude=norm_amplitude_mag;
    acc.norm_amplitude=norm_amplitude_acc;
    gyr.norm_amplitude=norm_amplitude_gyr;
    DataScaleForCalculations=3000.0;

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


}

//------------------------------------------------------------------------------------//

CalibrationGenerator::~CalibrationGenerator() {
}



//------------------------------------------------------------------------------------//

void CalibrationGenerator::InitialiseCalibrationObjectMag(Sensor3DCalibration & cal) {
    cal.SetBiasValues(mag.bias*DataScaleForCalculations);
    cal.SetSensitivityValues(mag.sensitivity*DataScaleForCalculations);
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


arma::vec CalibrationGenerator::GetMagCentroid(ImuDataset & set) {
    vec centroid(3,fill::zeros);
    vec min(3,fill::zeros);
    vec max(3,fill::zeros);

    if(set.size()>0) {
        min=set[0].Magnetometer;
        max=min;
    }
    for(size_t i=1; i<set.size(); i++) {
        if(set[i].Magnetometer(0)<min(0)) {
            min(0)=set[i].Magnetometer(0);
        }
        if(set[i].Magnetometer(1)<min(1)) {
            min(1)=set[i].Magnetometer(1);
        }
        if(set[i].Magnetometer(2)<min(2)) {
            min(2)=set[i].Magnetometer(2);
        }
        if(set[i].Magnetometer(0)>max(0)) {
            max(0)=set[i].Magnetometer(0);
        }
        if(set[i].Magnetometer(1)>max(1)) {
            max(1)=set[i].Magnetometer(1);
        }
        if(set[i].Magnetometer(2)>max(2)) {
            max(2)=set[i].Magnetometer(2);
        }
    }
    centroid=(min+max)/2.0;

    return centroid;
}

//------------------------------------------------------------------------------------//

arma::vec CalibrationGenerator::GetMagPlaneNormal(ImuDataset & set) {
    vec b(set.size());
    mat C(set.size(),3);

    //Fit data in plain formula
    for(size_t i=0; i<set.size(); i++) {
        b(i) = -set[i].Magnetometer(2);
        C(i,0) = set[i].Magnetometer(0);
        C(i,1) = set[i].Magnetometer(1);
        C(i,2) = 1.0;
    }

    arma::mat Q,R;
    arma::qr(Q,R,C);
    vec solution;
    arma::solve(solution,R,arma::trans(Q)*b);//solve(C,b);

    //get normal vector of plain
    vec normal(3);
    normal(0) = solution(0);
    normal(1) = solution(1);
    normal(2) = 1.0;

    //and normalize it
    return normal * (1.0/abs_vec(normal));
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::CalculateMagnetometerCalibrationData(ImuDataset & set) {
    vec b(set.size());
    mat C(set.size(), 9);
    double scale=DataScaleForCalculations;

    for(u32 i=0; i<set.size(); ++i) {
        b(i)   = 1;

        C(i,0) = set[i].Magnetometer(0)/scale * set[i].Magnetometer(0)/scale;
        C(i,1) = set[i].Magnetometer(1)/scale*set[i].Magnetometer(1)/scale;
        C(i,2) = set[i].Magnetometer(2)/scale*set[i].Magnetometer(2)/scale;
        C(i,3) = set[i].Magnetometer(0)/scale*set[i].Magnetometer(1)/scale*2.0;
        C(i,4) = set[i].Magnetometer(0)/scale*set[i].Magnetometer(2)/scale*2.0;
        C(i,5) = set[i].Magnetometer(1)/scale*set[i].Magnetometer(2)/scale*2.0;
        C(i,6) = set[i].Magnetometer(0)/scale*2.0;
        C(i,7) = set[i].Magnetometer(1)/scale*2.0;
        C(i,8) = set[i].Magnetometer(2)/scale*2.0;
    }

    arma::mat Q,R;
    arma::qr(Q,R,C);
    vec solution;
    arma::solve(solution,R,arma::trans(Q)*b);

    arma::mat A(4,4);


    //To matrix equation form
    A(0,0)=solution(0);
    A(1,1)=solution(1);
    A(2,2)=solution(2);
    A(3,3)=-1;
    A(0,1)=solution(3);
    A(1,0)=A(0,1);
    A(0,2)=solution(4);
    A(2,0)=A(0,2);
    A(1,2)=solution(5);
    A(2,1)=A(1,2);
    A(0,3)=solution(6);
    A(3,0)=A(0,3);
    A(1,3)=solution(7);
    A(3,1)=A(1,3);
    A(2,3)=solution(8);
    A(3,2)=A(2,3);


    arma::vec Center=A.submat(0,0,2,2).i()* ((-1)*(A.submat(0,3,2,3)));

    arma::mat TranslationMatrix(4,4,fill::eye);
    TranslationMatrix(3,0)=Center(0);
    TranslationMatrix(3,1)=Center(1);
    TranslationMatrix(3,2)=Center(2);


    arma::mat Translated=TranslationMatrix * A * TranslationMatrix.t();

    arma::mat CenteredEllipsoid=Translated.submat(0,0,2,2) * (-1.0/ Translated(3,3));

    arma::vec eigval;
    arma::mat eigvec;


    eig_sym(eigval,eigvec,CenteredEllipsoid);

    arma::mat eigval_root(3,3,fill::zeros);
    eigval_root(0,0)=sqrt(eigval(0));
    eigval_root(1,1)=sqrt(eigval(1));
    eigval_root(2,2)=sqrt(eigval(2));

    mag.combined=eigval_root*eigvec.t();
    mag.bias=Center;
    CalculateVectorsFromCombinedMatrix(mag);

}

//------------------------------------------------------------------------------------//

arma::vec CalibrationGenerator::GetAccVector(ImuDataset & set) {
    vec solution(3,fill::zeros);

    for(u32 i=0; i<set.size(); ++i) {
        solution+=set[i].Accelerometer;
    }

    return solution * (1.0/set.size());
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::CalculateAccelerometerCalibrationData(arma::vec zp, arma::vec zn, arma::vec xp, arma::vec xn, arma::vec yp, arma::vec yn) {
    size_t i=0;
    vec b(6*3);
    mat C(6*3, 12);



    help_buildEquationSystem(b,C,i,xp,arma::vec("1.0 0.0 0.0")*acc.norm_amplitude);
    help_buildEquationSystem(b,C,i,xn,arma::vec("-1.0 0.0 0.0")*acc.norm_amplitude);
    help_buildEquationSystem(b,C,i,yp,arma::vec("0.0 1.0 0.0")*acc.norm_amplitude);
    help_buildEquationSystem(b,C,i,yn,arma::vec("0.0 -1.0 0.0")*acc.norm_amplitude);
    help_buildEquationSystem(b,C,i,zp,arma::vec("0.0 0.0 1.0")*acc.norm_amplitude);
    help_buildEquationSystem(b,C,i,zn,arma::vec("0.0 0.0 -1.0")*acc.norm_amplitude);

    arma::mat Q,R;
    arma::qr(Q,R,C);
    vec solution;
    arma::solve(solution,R,arma::trans(Q)*b);

    vec bias=arma::vec(3);
    acc.bias(0)=solution(0);
    acc.bias(1)=solution(1);
    acc.bias(2)=solution(2);

    acc.combined(0,0)=solution(3);
    acc.combined(0,1)=solution(4);
    acc.combined(0,2)=solution(5);
    acc.combined(1,0)=solution(6);
    acc.combined(1,1)=solution(7);
    acc.combined(1,2)=solution(8);
    acc.combined(2,0)=solution(9);
    acc.combined(2,1)=solution(10);
    acc.combined(2,2)=solution(11);

    acc.bias=acc.combined.i()*acc.bias;
    CalculateVectorsFromCombinedMatrix(acc);

}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::CalculateGyrosocpeCalibrationData(ImuDataset & set) {
    //Not yet implemented
    gyr.bias(0)=0;
    gyr.bias(1)=0;
    gyr.bias(2)=0;
    gyr.combined(0,0)=1;
    gyr.combined(1,1)=1;
    gyr.combined(2,2)=1;
    CalculateVectorsFromCombinedMatrix(gyr);
}
/**************************************************************************************
 * PRIVATE FUNCTIONS
 **************************************************************************************/

static void help_buildEquationSystem(arma::vec & b, arma::mat &C, size_t & i, arma::vec input_vector, arma::vec output_vector) {


    //x-axis value
    b(i) = output_vector(0); //x-axis value
    C(i,0)=-1; //bx*a11+by*a12+bz*a13
    C(i,1)= 0; //bx*a21+by*a22+bz*a23
    C(i,2)= 0; //bx*a31+by*a32+bz*a33
    C(i,3)=input_vector(0); //a11
    C(i,4)=input_vector(1); //a12
    C(i,5)=input_vector(2); //a13
    C(i,6)=0; //a21
    C(i,7)=0; //a22
    C(i,8)=0; //a23
    C(i,9)=0; //a31
    C(i,10)=0;//a32
    C(i,11)=0;//a33
    i++;
    //y-axis value
    b(i) = output_vector(1); //y-axis value
    C(i,0)= 0; //bx*a11+by*a12+bz*a13
    C(i,1)= -1; //bx*a21+by*a22+bz*a23
    C(i,2)= 0; //bx*a31+by*a32+bz*a33
    C(i,3)=0; //a11
    C(i,4)=0; //a12
    C(i,5)=0; //a13
    C(i,6)=input_vector(0); //a21
    C(i,7)=input_vector(1); //a22
    C(i,8)=input_vector(2); //a23
    C(i,9)=0; //a31
    C(i,10)=0;//a32
    C(i,11)=0;//a33
    i++;
    //z-axis value
    b(i) = output_vector(2); //z-axis value
    C(i,0)= 0; //bx*a11+by*a12+bz*a13
    C(i,1)= 0; //bx*a21+by*a22+bz*a23
    C(i,2)= -1; //bx*a31+by*a32+bz*a33
    C(i,3)=0; //a11
    C(i,4)=0; //a12
    C(i,5)=0; //a13
    C(i,6)=0; //a21
    C(i,7)=0; //a22
    C(i,8)=0; //a23
    C(i,9)=input_vector(0); //a31
    C(i,10)=input_vector(1);//a32
    C(i,11)=input_vector(2);//a33
    i++;
}

//------------------------------------------------------------------------------------//

void CalibrationGenerator::CalculateVectorsFromCombinedMatrix(CalData & cal) {

    //Do some math magic to solve the equation system (coefficient comparision):
    //Combined_Matrice*(MeasuredDataVector-Bias) = EstimatedDataVector
    //(AlignementMatrice^-1)*(OrthogonalisationMatrice^-1)*(SensitivityMatrice^-1)*(MeasuredData-Bias)=EstimatedDataVector
    //For the elements of the matrices see (http://www.sciencedirect.com/science/article/pii/S0924424707003834)

    cal.alignement(2)=atan2(cal.combined(1,0),cal.combined(0,0));
    cal.alignement(1)=atan2(-cal.combined(2,0)*sin(cal.alignement(2)),cal.combined(1,0));
    cal.alignement(0)=atan2(((cal.combined(1,2)/cal.combined(2,2))-tan(cal.alignement(1))*sin(cal.alignement(2))),-cos(cal.alignement(2))/cos(cal.alignement(1)));

    cal.sensitivity(0)=cos(cal.alignement(1))*cos(cal.alignement(2))/cal.combined(0,0);
    cal.sensitivity(1)=sin(cal.alignement(0))*cos(cal.alignement(1))/cal.combined(2,1);
    cal.sensitivity(2)=cos(cal.alignement(0))*cos(cal.alignement(1))/cal.combined(2,2);

    //Internal orthogonalisation cannot be estimated by the used calibration process, so estimate it to be ideal (90 deg)
    cal.orthogonalisation(0)=M_PI/2.0;
    cal.orthogonalisation(1)=M_PI/2.0;
    cal.orthogonalisation(2)=M_PI/2.0;
}

