/*
 * CalibrationGeneration.h
 *
 *  Created on: 14.12.2015
 *      Author: robocop
 */

#ifndef CALIBRATIONGENERATION_H_
#define CALIBRATIONGENERATION_H_
#include <armadillo>
#include "../../process_imu_data/src/Sensor3DCalibration.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <vector>



class CalibrationGeneration {
public:
	CalibrationGeneration(double norm_amplitude_mag,double norm_amplitude_acc, double norm_amplitude_gyr);
	virtual ~CalibrationGeneration();

	void CalibrationStep(arma::vec const & input_mag,arma::vec const & input_acc,arma::vec const & input_gyr);
	void InitialiseCalibrationObjectMag(Sensor3DCalibration & cal);
	void InitialiseCalibrationObjectAcc(Sensor3DCalibration & cal);
	void InitialiseCalibrationObjectGyr(Sensor3DCalibration & cal);
	bool isFinished();
	void progressStep();
private:
	
	bool proceed;
	
	struct SolutionEntry {		
		arma::vec Measured;
		arma::vec Estimated;
	};
	
	std::vector<SolutionEntry> mag_data,acc_data, gyr_data;
	
	struct CalData{
		double norm_amplitude;
		arma::mat combined;
		arma::vec bias;
		arma::vec sensitivity;
		arma::vec orthogonalisation;
		arma::vec alignement;
	} mag, acc, gyr;
	
	void CalculateVectorsFromCombinedMatrix(CalData & cal);
	
	enum {
		st_IDLE,
		st_ZP,
		st_PAUSE1,
		st_ZN,
		st_PAUSE2,
		st_XP,
		st_PAUSE3,
		st_XN,
		st_PAUSE4,
		st_YP,
		st_PAUSE5,
		st_YN,
		st_SPERE,
		st_FINISHED
	} calState;
};

#endif /* CALIBRATIONGENERATION_H_ */
