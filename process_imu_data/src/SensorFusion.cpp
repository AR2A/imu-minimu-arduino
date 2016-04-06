/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     SensorFusion.cpp
 * \license  BSD-3-License
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <math.h>

#include <boost/filesystem.hpp>

#include "SensorFusion.h"

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

SensorFusion::SensorFusion(std::string const & path, double deltaT) {
    m_Filename=path+std::string("/")+std::string("kalman_");

    boost::filesystem::create_directories(path);

    //Input coupling is set to zero angle in each direction
    m_kalman_InputCoupling=mat(3,3,fill::zeros);
    m_kalman_InputCoupling(0,0)=0;
    m_kalman_InputCoupling(1,0)=0;
    m_kalman_InputCoupling(2,0)=1;

    m_kalman_InputCoupling(0,1)=0;
    m_kalman_InputCoupling(1,1)=1;
    m_kalman_InputCoupling(2,1)=0;

    m_kalman_InputCoupling(0,2)=1;
    m_kalman_InputCoupling(1,2)=0;
    m_kalman_InputCoupling(2,2)=0;


    if(!m_kalman_StateTransition.load(m_Filename+"stateTransition.mat",raw_ascii)) {
        m_kalman_StateTransition=mat(3,3,fill::eye);
        m_kalman_StateTransition.save(m_Filename+"stateTransition.mat",raw_ascii);
    }

    if(!m_kalman_MeasurementSensitivity.load(m_Filename+"measurementSensitivity.mat",raw_ascii)) {
        m_kalman_MeasurementSensitivity=mat(3,3,fill::eye);
        m_kalman_MeasurementSensitivity.save(m_Filename+"measurementSensitivity.mat",raw_ascii);
    }

    m_kalman_AngleEstimation[0]=arma::vec(3,fill::zeros);
    m_kalman_AngleEstimation[1]=arma::vec(3,fill::zeros);
    m_kalman_ErrorCovariance[0]=arma::mat(3,3,fill::eye);
    m_kalman_ErrorCovariance[1]=arma::mat(3,3,fill::eye);

    m_kalman_ProcessNoiseCovariance=arma::mat(3,3,fill::eye);
    m_kalman_MeasurementNoiseCovariance=arma::mat(3,3,fill::eye);

    m_deltaT=deltaT;
    m_currentDataset=0;
}

//------------------------------------------------------------------------------------//

SensorFusion::~SensorFusion() {

}

//------------------------------------------------------------------------------------//

SensorFusion::FusedData SensorFusion::operator()(arma::vec const & gyro,arma::vec const & acc,arma::vec const & mag) {
    FusedData return_value;
    kalman_update(gyro);

    if(isDataPlausible(acc,mag)) {

        arma::vec measured_state(3,fill::zeros);

        //Calculate bank and elevation angle from accelerometer data
        measured_state(2)= atan2(acc(1),acc(2));
        measured_state(1) = atan2(-acc(0),sqrt(acc(1)*acc(1)+acc(2)*acc(2)));

        //tilt compensate magnetometer data with calculated angles
        arma::mat rot(3,3,fill::eye);
        rot(0,0) = cos(measured_state(1));
        rot(1,1) = -cos(measured_state(2));
        rot(2,0) = -sin(measured_state(1));
        rot(1,2) = -sin(measured_state(2));
        rot(0,1) = rot(2,0)*rot(1,2);
        rot(0,2) = rot(2,0)*rot(1,1);
        rot(1,0) = 0;
        rot(2,1) = (-rot(1,2))*rot(0,0);
        rot(2,2) = rot(0,0)*(-rot(1,1));
        arma::vec mag_rot= rot*mag;

        //Calculate heading angle with compensated magnetometer data;
        measured_state(0)= atan2(-mag_rot(1),mag_rot(0));


        calculate_MeasurementNoiseCovariance(acc,mag);
        //execute correction step
        kalman_correct(measured_state);
    }

    calculate_ProcessNoiseCovariance(gyro);
    return_value.angles=m_kalman_AngleEstimation[m_currentDataset];
    return_value.covariance=m_kalman_ErrorCovariance[m_currentDataset];
    kalman_finish();

    return return_value;
}

/**************************************************************************************
 * PRIVATE FUNCTIONS
 **************************************************************************************/

bool SensorFusion::isDataPlausible(arma::vec const & acc,arma::vec const & mag) {
    double diff_acc=abs_vec(acc)-9.81;
    double diff_mag=abs_vec(mag)-1;

    const double acc_trust_interval=0.1;
    const double mag_trust_interval=0.01;

    return (diff_acc<acc_trust_interval && diff_mag<mag_trust_interval);
}

//------------------------------------------------------------------------------------//

void SensorFusion::calculate_MeasurementNoiseCovariance(arma::vec const & acc,arma::vec const & mag) {
    const double acc_trust_interval=0.1;
    const double acc_sigma_lo=10;
    const double acc_sigma_hi=100;
    const double mag_trust_interval=0.01;
    const double mag_sigma_lo=10;
    const double mag_sigma_hi=100;

    bool mag_val=false;
    bool acc_val=false;

    double diff_acc=abs_vec(acc)-9.81;
    double diff_mag=abs_vec(mag)-1;

    if(diff_acc<0) {
        diff_acc=-diff_acc;
    }
    if(diff_mag<0) {
        diff_mag=-diff_mag;
    }

    m_kalman_MeasurementNoiseCovariance(0,0)=mag_sigma_lo+diff_mag*(mag_sigma_hi-mag_sigma_lo)/mag_trust_interval;
    m_kalman_MeasurementNoiseCovariance(1,1)=acc_sigma_lo+diff_acc*(acc_sigma_hi-acc_sigma_lo)/acc_trust_interval;
    m_kalman_MeasurementNoiseCovariance(2,2)=m_kalman_MeasurementNoiseCovariance(1,1);
}

//------------------------------------------------------------------------------------//

void SensorFusion::calculate_ProcessNoiseCovariance(arma::vec const & gyro) {
    const double gyro_sigma_lo=0.01;
    const double gyro_sigma_hi=0.05;
    const double gyro_max=0.785398; //45 degree   //OLD: //4.53786; //260 degree
    double amount_speed_change=abs_vec(gyro);

    m_kalman_ProcessNoiseCovariance(0,0)= gyro_sigma_lo  + amount_speed_change*(gyro_sigma_hi-gyro_sigma_lo)/gyro_max;
    m_kalman_ProcessNoiseCovariance(1,1)=m_kalman_ProcessNoiseCovariance(0,0);
    m_kalman_ProcessNoiseCovariance(2,2)=m_kalman_ProcessNoiseCovariance(0,0);

}

//------------------------------------------------------------------------------------//

void SensorFusion::kalman_update(arma::vec const & gyro) {
    size_t lastDataset=(m_currentDataset+c_NrStoredDatasets-1)%c_NrStoredDatasets;

    //Model of the ideal system: NewAngles=UnitMatrix*LastAngles+InputCoupling*AngleDifference
    m_kalman_AngleEstimation[m_currentDataset]=
        m_kalman_StateTransition*m_kalman_AngleEstimation[lastDataset]+
        m_kalman_InputCoupling*gyro*m_deltaT;

    //Model of the error of the ideal system
    m_kalman_ErrorCovariance[m_currentDataset]=
        m_kalman_StateTransition*m_kalman_ErrorCovariance[lastDataset]*
        m_kalman_StateTransition.t()+m_kalman_ProcessNoiseCovariance;

}

//------------------------------------------------------------------------------------//

void SensorFusion::kalman_correct(arma::vec const & measuredState) {
    //Kalman Gain
    arma::mat kalmanGain = m_kalman_ErrorCovariance[m_currentDataset]*m_kalman_MeasurementSensitivity.t()*
                           (m_kalman_MeasurementSensitivity*m_kalman_ErrorCovariance[m_currentDataset]*m_kalman_MeasurementSensitivity.t() + m_kalman_MeasurementNoiseCovariance).i();


    //Correct Estimation
    m_kalman_AngleEstimation[m_currentDataset]=m_kalman_AngleEstimation[m_currentDataset]+
            kalmanGain*(measuredState-m_kalman_MeasurementSensitivity*m_kalman_AngleEstimation[m_currentDataset]);

    //Correct Error
    m_kalman_ErrorCovariance[m_currentDataset]=(eye<mat>(3,3)-kalmanGain*m_kalman_MeasurementSensitivity)*m_kalman_ErrorCovariance[m_currentDataset];
}

//------------------------------------------------------------------------------------//

void SensorFusion::kalman_finish() {
    size_t lastDataset=(m_currentDataset+c_NrStoredDatasets-1)%c_NrStoredDatasets;
    size_t nextDataset=(m_currentDataset+1)%c_NrStoredDatasets;

    //Calculate new InputCouplingMatrix (Rotate angular velocity)
    m_kalman_InputCoupling(0,0)=0;
    m_kalman_InputCoupling(1,0)=0;
    m_kalman_InputCoupling(2,0)=1;
    if(cos(m_kalman_AngleEstimation[m_currentDataset](1))==0.0) {
        //Modify anle in case of gimbal lock - (introduces a slight error but a rotation around the y axis of 90 deg should never ever occur with a driving vehicle)
        //And if it occurs the vehicle has no need to navigate any further cause it cant drive on the side ;)
        //If your vehicle can - you have to do the body frame angular rate to euler conversion as quaternion.
        m_kalman_AngleEstimation[m_currentDataset](1)+=0.01*(m_kalman_AngleEstimation[m_currentDataset](1)-m_kalman_AngleEstimation[lastDataset](1));
    }
    m_kalman_InputCoupling(0,1)=sin(m_kalman_AngleEstimation[m_currentDataset](2))/cos(m_kalman_AngleEstimation[m_currentDataset](1));
    m_kalman_InputCoupling(1,1)=cos(m_kalman_AngleEstimation[m_currentDataset](2));
    m_kalman_InputCoupling(2,1)=sin(m_kalman_AngleEstimation[m_currentDataset](2))*tan(m_kalman_AngleEstimation[m_currentDataset](1));

    m_kalman_InputCoupling(0,2)=cos(m_kalman_AngleEstimation[m_currentDataset](2))/cos(m_kalman_AngleEstimation[m_currentDataset](1));
    m_kalman_InputCoupling(1,2)=-sin(m_kalman_AngleEstimation[m_currentDataset](2));
    m_kalman_InputCoupling(2,2)=cos(m_kalman_AngleEstimation[m_currentDataset](2))*tan(m_kalman_AngleEstimation[m_currentDataset](1));

    m_currentDataset = nextDataset;

}
