/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     SensorFusion.h
 * \license  BSD-3-License
 */

#ifndef SENSORFUSION_H_
#define SENSORFUSION_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>

#include <armadillo>

/**************************************************************************************
 * CLASSES
 **************************************************************************************/

class SensorFusion {
  public:
    /**
     * @brief Datatype to describe the fused sensor data
     */
    struct FusedData {
        arma::vec angles;		/**< */
        arma::mat covariance;	/**< */
    };

    /**
     * @brief Initializes the filter with the data stored in path.
     *
     * @param[in] path Filesystem path to the folder where the calibrated matrices are stored.
     * @param[in] deltaT Defines the sample rate of the filter - used to estimate the angles by the gyro rates.
     */
    SensorFusion(std::string const & path, double deltaT);

    /**
     * @brief Destructor.
     * Does nothing.
     */
    virtual ~SensorFusion();

    /**
     * @brief Performs on step of the kalman filter.
     * @note This Function has to be called with the refresh rate given by \ref m_deltaT.
     *
     * Performs the following steps:
     * * Kalman-Update \ref kalman_update()
     * * Checks wheter accelerometer data and magnetometer data are plausible \ref isDataPlausible()
     * 	- If they are, the magnetometer data is tilt corrected and angles are calculated with the data,
     * 	- Measurement noise is estimated \ref calculate_MeasurementNoiseCovariance()
     * 	- And Kalman-Correction is performed \ref kalman_correct()
     * * Process noise is estimated \ref calculate_ProcessNoiseCovariance()
     * * Preparations for the next call are made \ref kalman_finish()
     *
     * @param[in] gyro Gyroscope data in the following format: \f$ \vec{\omega}= \begin{bmatrix} \omega_z \\ \omega_y \\ \omega_x \end{bmatrix} \frac{rad}{s} \f$
     * @param[in] acc Linear acceleration in the following format: \f$ \vec{a}= \begin{bmatrix} a_x \\ a_y \\ a_z \end{bmatrix} \frac{m}{s^2} \f$
     * @param[in] mag Magnetic field in the following format: \f$ \vec{B}= \begin{bmatrix} B_x \\ B_y \\ B_z \end{bmatrix} T \f$
     * @return The current attitude and heading in the following format \f$\vec{a}= \begin{bmatrix}heading\\elevation\\bank \end{bmatrix} rad \f$ alongside
     * 	with the corresponding covariance as 3 by 3 matrix.
     */
    FusedData operator()(arma::vec const & gyro,arma::vec const & acc,arma::vec const & mag);


  private:
    static size_t const c_NrStoredDatasets=2; ///< Number of passed data to store (at minimum 2)

    /**
     * @brief Checks whether accelerometer and magnetometer data are in a plausible range to derive an angle.
     *
     * Accelerometer data is plausible if the absolute length of the vector resembles gravitational acceleration (g),
     * because the bank and elevation angles are calculated from the g-vector.
     *
     * Magnetometer data is plausible if the normalized data is nearly on the unit sphere,
     * because it's data is used to calculate the heading angle.
     *
     * @param[in] acc Linear acceleration in the following format: \f$ \vec{a}= \begin{bmatrix} a_x \\ a_y \\ a_z \end{bmatrix} \frac{m}{s^2} \f$
     * @param[in] mag Magnetic field in the following format: \f$ \vec{B}= \begin{bmatrix} B_x \\ B_y \\ B_z \end{bmatrix} T \f$
     *
     * @return True or false whether the data is plausible to use for further calculations.
     */
    bool isDataPlausible(arma::vec const & acc,arma::vec const & mag);

    /**
     * @brief Calculates the measurement noise covariance from accelerometer and magnetometer data.
     *
     * The measurement covariance is used in the correction step of the kalman-filter,
     * it is determined by the following equation:
     *	\f$ R = \begin{bmatrix}
     *	 \sigma_{\Psi lo}+\delta_{mag}*\frac{\sigma_{\Psi hi}-\sigma_{\Psi lo}}{T_{mag}} & 0 & 0 \\
     *	 0 & \sigma_{\Theta lo}+\delta_{acc}*\frac{\sigma_{\Theta hi}-\sigma_{\Theta lo}}{T_{acc}} & 0 \\
     *	 0 & 0 & \sigma_{\Phi lo}+\delta_{acc}*\frac{\sigma_{\Phi hi}-\sigma_{\Phi lo}}{T_{acc}}\\
     *	\end{bmatrix}\f$
     */
    void calculate_MeasurementNoiseCovariance(arma::vec const & acc,arma::vec const & mag);

    /**
     * @brief
     * @param[in] gyro
     */
    void calculate_ProcessNoiseCovariance(arma::vec const & gyro);

    /**
     * @brief
     * @param[in] gyro
     */
    void kalman_update(arma::vec const & gyro);

    /**
     * @brief
     * @param[in]
     */
    void kalman_correct(arma::vec const & measuredState);

    /**
     * @brief
     */
    void kalman_finish();


    arma::vec m_kalman_AngleEstimation[c_NrStoredDatasets]; /**< */
    arma::mat m_kalman_ErrorCovariance[c_NrStoredDatasets]; /**< */
    arma::mat m_kalman_StateTransition;						/**< */
    arma::mat m_kalman_MeasurementSensitivity;				/**< */
    arma::mat m_kalman_InputCoupling;						/**< */
    arma::mat m_kalman_ProcessNoiseCovariance;				/**< */
    arma::mat m_kalman_MeasurementNoiseCovariance;			/**< */

    size_t m_currentDataset;								/**< */
    double m_deltaT;										/**< */
    std::string m_Filename;									/**< */
};

#endif /* SENSORFUSION_H_ */
