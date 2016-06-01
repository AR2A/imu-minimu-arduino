/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     CalibrationGenerator.h
 * \license  BSD-3-License
 */

#ifndef CALIBRATIONGENERATION_H_
#define CALIBRATIONGENERATION_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <vector>

#include <armadillo>

#include "../../process_imu_data/src/Sensor3DCalibration.h"

/**************************************************************************************
 * CLASSES
 **************************************************************************************/
class ImuData {
  public:
    arma::vec Magnetometer;
    arma::vec Accelerometer;
    arma::vec Gyroscope;
};

typedef std::vector<ImuData> ImuDataset;

/**
 * @brief This class is responsible to calculate the calibration matrices for an imu.
 *	Calculation is achieved with a statemachine which instructs the user to align the
 *	imu to certain known orientations - to store the measured and the expected data.
 *	If every orientation is handled the resulting system is estimated by an least squares
 *	solver.
 */
class CalibrationGenerator {
  public:

    /**
     * @brief Constructor
     * Initializes the zero position amplitudes of the sensors
     * @param[in] norm_amplitude_mag zero position amplitude of the magnetometer
     * @param[in] norm_amplitude_acc zero position amplitude of the accelerometer
     * @param[in] norm_amplitude_gyr zero position amplitude of the gyroscope
     */
    CalibrationGenerator(double norm_amplitude_mag,double norm_amplitude_acc, double norm_amplitude_gyr);

    /**
     * @brief Destructor
     */
    virtual ~CalibrationGenerator();

    /**
     * @brief Writes the calculated calibration data for the magnetometer to an calibration object.
     * @param[in] cal The calibration object which shall store the data
     */
    void InitialiseCalibrationObjectMag(Sensor3DCalibration & cal);

    /**
     * @brief Writes the calculated calibration data for the accelerometer to an calibration object.
     * @param[in] cal The calibration object which shall store the data
     */
    void InitialiseCalibrationObjectAcc(Sensor3DCalibration & cal);

    /**
     * @brief Writes the calculated calibration data for the gyroscope to an calibration object.
     * @param[in] cal The calibration object which shall store the data
     */
    void InitialiseCalibrationObjectGyr(Sensor3DCalibration & cal);



    arma::vec GetMagCentroid(ImuDataset & set);
    arma::vec GetMagPlaneNormal(ImuDataset & set);
    arma::vec GetAccVector(ImuDataset & set);
    /**
     * @brief calculates the calibration factors from a dataset containing rotations around all axes(sphere)
     */
    void CalculateMagnetometerCalibrationData(ImuDataset & set);
    /**
     * @brief calculates the calibration factors from six vectors describing the coordinate system
     */
    void CalculateAccelerometerCalibrationData(arma::vec zp, arma::vec zn, arma::vec xp, arma::vec xn, arma::vec yp, arma::vec yn);
    void CalculateGyrosocpeCalibrationData(ImuDataset & set);
  private:

    double DataScaleForCalculations;


    /**
     * Used to store intermediate calibration data
     */
    struct CalData {
        double norm_amplitude;       /**< zero position amplitude*/
        arma::mat combined;          /**< merged calibration matrix (no semantics)*/
        arma::vec bias;              /**< bias vector*/
        arma::vec sensitivity;       /**< sensitivity vector (scaling) - derived from combined matrix*/
        arma::vec orthogonalisation; /**< orthogonalisation vector (axis angles) - derived from combined matrix*/
        arma::vec alignement;        /**< alignement vector (alignement of whole sensor) - derived from combined matrix*/
    } mag, acc, gyr;

    /**
     * @brief Internal function to derive calibration parameters from combined matrix
     * @param[inout] cal Calibration data structure containing a filled combined matrix - returned with all parameters filled
     */
    void CalculateVectorsFromCombinedMatrix(CalData & cal);
};

#endif // CALIBRATIONGENERATION_H_ 
