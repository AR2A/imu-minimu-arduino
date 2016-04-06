/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     Sensor3DCalibration..h
 * \license  BSD-3-License
 */

#ifndef SENSOR3DCALIBRATION_H_
#define SENSOR3DCALIBRATION_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>

#include <armadillo>

/**************************************************************************************
 * CLASSES
 **************************************************************************************/

class Sensor3DCalibration {
  public:
    /**
     * @brief
     * @param[in] path
     * @param[in] prefix
     */
    Sensor3DCalibration(std::string const & path, std::string const & prefix);
    /**
     * @brief
     */
    virtual ~Sensor3DCalibration();

    /**
     * @brief
     * @param[in] input
     * @return
     */
    arma::vec  operator()(arma::vec const & input);

    /**
     * @brief
     * @param[in] bias
     */
    void SetBiasValues(arma::vec const & bias);

    /**
     * @brief
     * @param[in] sensitivity
     */
    void SetSensitivityValues(arma::vec const & sensitivity);

    /**
     * @brief
     * @param[in] orthogonalisation
     */
    void SetOrthogonalisationValues(arma::vec const & orthogonalisation);

    /**
     * @brief
     * @param[in] alignement
     */
    void SetAlignementValues(arma::vec const & alignement);

  private:

    /**
     * @brief
     */
    void updateCombinedMatrix();

    arma::mat  m_AlignementMatrix;			/**< */
    arma::mat  m_OrthogonalisationMatrix;	/**< */
    arma::mat  m_SensitivityMatrix;			/**< */
    arma::mat  m_CombinedMatrix;			/**< */
    arma::vec  m_BiasVector;				/**< */

    std::string m_Filename;					/**< */
};

#endif // SENSOR3DCALIBRATION_H_ 
