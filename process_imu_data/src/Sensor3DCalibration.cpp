/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     Sensor3DCalibration.cpp
 * \license  BSD-3-License
 */

/**************************************************************************************
* INCLUDES
**************************************************************************************/

#include <math.h>
#include <sstream>

#include <boost/filesystem.hpp>

#include "Sensor3DCalibration.h"

using namespace arma;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

/**************************************************************************************
 * PUBLIC FUNCTIONS
 **************************************************************************************/

Sensor3DCalibration::Sensor3DCalibration(std::string const & path, std::string const & prefix) {
    m_Filename=path+std::string("/")+prefix+std::string("_");

    boost::filesystem::create_directories(path);

    //Load or init alignement
    if(!m_AlignementMatrix.load(m_Filename+"alignement.mat",raw_ascii)) {
        m_AlignementMatrix=mat(3,3,fill::eye);
        m_AlignementMatrix.save(m_Filename+"alignement.mat",raw_ascii);
    }

    //Load or init orthogonalisation
    if(!m_OrthogonalisationMatrix.load(m_Filename+"orthogonalisation.mat",raw_ascii)) {
        m_OrthogonalisationMatrix=mat(3,3,fill::eye);
        m_OrthogonalisationMatrix.save(m_Filename+"orthogonalisation.mat",raw_ascii);
    }

    //Load or init sensitivity
    if(!m_SensitivityMatrix.load(m_Filename+"sensitivity.mat",raw_ascii)) {
        m_SensitivityMatrix=mat(3,3,fill::eye);
        m_SensitivityMatrix.save(m_Filename+"sensitivity.mat",raw_ascii);
    }

    //Load or init bias
    if(!m_BiasVector.load(m_Filename+"bias.mat",raw_ascii)) {
        m_BiasVector=vec(3,fill::zeros);
        m_BiasVector.save(m_Filename+"bias.mat",raw_ascii);
    }

    updateCombinedMatrix();
}

//------------------------------------------------------------------------------------//

Sensor3DCalibration::~Sensor3DCalibration() {

}

//------------------------------------------------------------------------------------//

vec  Sensor3DCalibration::operator()(vec const & input) {
    return m_CombinedMatrix*(input-m_BiasVector);
}

//------------------------------------------------------------------------------------//

void Sensor3DCalibration::SetBiasValues(arma::vec const & bias) {
    m_BiasVector=bias;
    m_BiasVector.save(m_Filename+"bias.mat",raw_ascii);
}

//------------------------------------------------------------------------------------//

void Sensor3DCalibration::SetSensitivityValues(arma::vec const & sensitivity) {
    m_SensitivityMatrix(0,0)=sensitivity(0);
    m_SensitivityMatrix(1,1)=sensitivity(1);
    m_SensitivityMatrix(2,2)=sensitivity(2);
    m_SensitivityMatrix.save(m_Filename+"sensitivity.mat",raw_ascii);
    updateCombinedMatrix();
}

//------------------------------------------------------------------------------------//

void Sensor3DCalibration::SetOrthogonalisationValues(arma::vec const & orthogonalisation) {
    m_OrthogonalisationMatrix=mat(3,3,fill::eye);
    m_OrthogonalisationMatrix(1,0)=cos(orthogonalisation(0));
    m_OrthogonalisationMatrix(2,0)=cos(orthogonalisation(1));
    m_OrthogonalisationMatrix(2,1)=cos(orthogonalisation(2));
    m_OrthogonalisationMatrix.save(m_Filename+"orthogonalisation.mat",raw_ascii);
    updateCombinedMatrix();
}

//------------------------------------------------------------------------------------//

void Sensor3DCalibration::SetAlignementValues(arma::vec const & alignement) {
    mat rotRoll(3,3,fill::eye);
    mat rotPitch(3,3,fill::eye);
    mat rotYaw(3,3,fill::eye);

    rotRoll(1,1)=cos(alignement(0));
    rotRoll(1,2)=sin(alignement(0));
    rotRoll(2,1)=-rotRoll(1,2);
    rotRoll(2,2)=rotRoll(1,1);

    rotPitch(0,0)=cos(alignement(1));
    rotPitch(2,0)=sin(alignement(1));
    rotPitch(0,2)=-rotPitch(2,0);
    rotPitch(2,2)=rotPitch(0,0);

    rotYaw(0,0)=cos(alignement(2));
    rotYaw(0,1)=sin(alignement(2));
    rotYaw(1,0)=-rotYaw(0,1);
    rotYaw(1,1)=rotYaw(0,0);

    m_AlignementMatrix=rotRoll*rotPitch*rotYaw;

    m_AlignementMatrix.save(m_Filename+"alignement.mat",raw_ascii);
    updateCombinedMatrix();
}

/**************************************************************************************
 * PRIVATE FUNCTIONS
 **************************************************************************************/

void Sensor3DCalibration::updateCombinedMatrix() {
    m_CombinedMatrix=m_AlignementMatrix.i()*m_OrthogonalisationMatrix.i()*m_SensitivityMatrix.i();
}
