/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     CalibrationGeneration.h
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

class CalibrationGeneration {
  public:
  
	/**
	 * @brief
	 * @param[in] norm_amplitude_mag
	 * @param[in] norm_amplitude_acc
	 * @param[in] norm_amplitude_gyr
	 */
    CalibrationGeneration(double norm_amplitude_mag,double norm_amplitude_acc, double norm_amplitude_gyr);
	
	/**
	 * @brief
	 */
    virtual ~CalibrationGeneration();

	/**
	 * @brief
	 * @param[in] input_mag
	 * @param[in] input_acc
	 * @param[in] input_gyr
	 */
    void CalibrationStep(arma::vec const & input_mag,arma::vec const & input_acc,arma::vec const & input_gyr);
	
	/**
	 * @brief
	 * @param[in] cal
	 */
    void InitialiseCalibrationObjectMag(Sensor3DCalibration & cal);
	
	/**
	 * @brief
	 * @param[in] cal
	 */
    void InitialiseCalibrationObjectAcc(Sensor3DCalibration & cal);
	
	/**
	 * @brief
	 * @param[in] cal
	 */
    void InitialiseCalibrationObjectGyr(Sensor3DCalibration & cal);
	
	/**
	 * @brief
	 */
    bool isFinished();
	
	/**
	 * @brief
	 */
    void progressStep();
  private:


  
    bool proceed; /**< */

	/**
	 *
	 */
    struct SolutionEntry {
        arma::vec Measured; /**< */
        arma::vec Estimated;/**< */
    };

    std::vector<SolutionEntry> mag_data; /**< */
	std::vector<SolutionEntry> acc_data; /**< */
	std::vector<SolutionEntry> gyr_data; /**< */
	
	/**
	 *
	 */
    struct CalData {
        double norm_amplitude;			/**< */
        arma::mat combined;				/**< */
        arma::vec bias;					/**< */
        arma::vec sensitivity;			/**< */
        arma::vec orthogonalisation;	/**< */
        arma::vec alignement;			/**< */
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
