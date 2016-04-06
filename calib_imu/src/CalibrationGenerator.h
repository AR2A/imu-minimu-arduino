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
     * @brief Perfors a single step of the calibration calculation
     * @param[in] input_mag Raw magnetometer reading
     * @param[in] input_acc Raw accelerometer reading
     * @param[in] input_gyr Raw gyroscope reading
     */
    void CalibrationStep(arma::vec const & input_mag,arma::vec const & input_acc,arma::vec const & input_gyr);

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

    /**
     * @brief Indicates whether the calculation of calibration data has finished
     */
    bool isFinished();

    /**
     * @brief Is an indication for the internal statemachine that the user changed the orientation of the imu
     */
    void doUserInput();
  private:
  
    bool proceed; /**< */

    /**
     * A single dataset with measured data as well as expected data
     */
    struct SolutionEntry {
        arma::vec Measured; /**< measured data from one sensor*/
        arma::vec Estimated;/**< corresponding expected data */
    };

    std::vector<SolutionEntry> mag_data; /**< */
    std::vector<SolutionEntry> acc_data; /**< */
    std::vector<SolutionEntry> gyr_data; /**< */

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
     *
     */
    enum {
        st_IDLE,	/**< */
        st_ZP,		/**< */
        st_PAUSE1,	/**< */
        st_ZN,		/**< */
        st_PAUSE2,	/**< */
        st_XP,		/**< */
        st_PAUSE3,	/**< */
        st_XN,		/**< */
        st_PAUSE4,	/**< */
        st_YP,		/**< */
        st_PAUSE5,	/**< */
        st_YN,		/**< */
        st_SPERE,	/**< */
        st_FINISHED	/**< */
    } calState;

    /**
     * @brief
     * @param[in] cal
     */
    void CalculateVectorsFromCombinedMatrix(CalData & cal);
};

#endif // CALIBRATIONGENERATION_H_ 
