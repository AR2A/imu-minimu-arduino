/*
 * Sensor3DCalibration.h
 *
 *  Created on: 14.12.2015
 *      Author: robocop
 */

#ifndef SENSOR3DCALIBRATION_H_
#define SENSOR3DCALIBRATION_H_
#include <armadillo>
#include <string>

class Sensor3DCalibration {
public:
	Sensor3DCalibration(std::string const & path, std::string const & prefix);
	virtual ~Sensor3DCalibration();

	arma::vec  operator()(arma::vec const & input);
	void SetBiasValues(arma::vec const & bias);
	void SetSensitivityValues(arma::vec const & sensitivity);
	void SetOrthogonalisationValues(arma::vec const & orthogonalisation);
	void SetAlignementValues(arma::vec const & alignement);

private:

	void updateCombinedMatrix();

	arma::mat  m_AlignementMatrix;
	arma::mat  m_OrthogonalisationMatrix;
	arma::mat  m_SensitivityMatrix;
	arma::mat  m_CombinedMatrix;
	arma::vec  m_BiasVector;

	std::string m_Filename;
};

#endif /* SENSOR3DCALIBRATION_H_ */
