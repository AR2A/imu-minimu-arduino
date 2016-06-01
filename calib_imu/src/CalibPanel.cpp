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
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>

#include <rviz/helpers/color.h>
#include <rviz/visualization_manager.h>


#include <armadillo>

#include "CalibrationGenerator.h"
#include "../../process_imu_data/src/Sensor3DCalibration.h"
#include "CalibPanel.h"
#include "CalibDisplay.h"

using namespace std;
using namespace arma;

/**************************************************************************************
 * TYPES
 **************************************************************************************/

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu ,sensor_msgs::MagneticField> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

/**
 * @brief Path where the calibration matrices are stored.
 * @todo Introduce a ros parameter to make this value configurable
 */
static string const PATH_TO_CALIBRATION = "./calibration";

static size_t const QUEUE_LENGTH=5; /**< Length of the queues attached to the message buffers of subsrcibed messages*/


namespace calib_imu {

/**************************************************************************************
 * INTERNAL CLASSES
 **************************************************************************************/

class SubscriberWrapper {
  public:
    SubscriberWrapper(ros::NodeHandle & nh, std::string  topic) {

        //Create the subscribers for both imu porovided messages
        sub_imu_data =
            new message_filters::Subscriber<sensor_msgs::Imu>(nh,ros::names::resolve(topic) + "/data_raw", QUEUE_LENGTH);

        sub_mag_data =
            new message_filters::Subscriber<sensor_msgs::MagneticField>(nh,ros::names::resolve(topic) + "/magnetic_field", QUEUE_LENGTH);

        //Synchronize both messages to the same timebase (they should be sent by the imu at the 'same' time.)
        sync = new Synchronizer(SyncPolicy(QUEUE_LENGTH),*sub_imu_data,*sub_mag_data);
    }

    ~SubscriberWrapper() {
        delete sync;
        delete sub_mag_data;
        delete sub_imu_data;

    }

    Synchronizer * GetSynchronizer() {
        return sync;
    }

  private:

    message_filters::Subscriber<sensor_msgs::Imu> * sub_imu_data;
    message_filters::Subscriber<sensor_msgs::MagneticField> * sub_mag_data;
    Synchronizer * sync;


};

//------------------------------------------------------------------------------------//

class CalibPanelPrivate {
  public:
    ImuDataset currentDataset;
    struct DisplayOptions {
        bool showData;
        rviz::Color color;
        double scale;
    };
    DisplayOptions displayMagnetometer;
    DisplayOptions displayAccelerometer;
    DisplayOptions displayGyroscope;
    bool recordData;

    enum State {
        Idle,
        pZPlane,
        nZPlane,
        pXPlane,
        nXPlane,
        pYPlane,
        nYPlane,
        Sphere
    } CalibState,PreviousState;
};
/**************************************************************************************
 * PUBLIC FUNCTIONS
 **************************************************************************************/

CalibPanel::CalibPanel( QWidget* parent )
    : rviz::Panel( parent ) {
    // Next we lay out the "output topic" text entry field using a
    // QLabel and a QLineEdit in a QHBoxLayout.
    QHBoxLayout* topic_layout = new QHBoxLayout;
    topic_layout->addWidget( new QLabel( "Read Topic:" ));
    read_topic_editor = new QLineEdit("imu");
    btn_proceed = new QPushButton("Record positive Z-Plane",this);

    topic_layout->addWidget( read_topic_editor );

    // Lay out the topic field above the control widget.
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( topic_layout );
    layout->addWidget( btn_proceed );
    setLayout( layout );

    // Next we make signal/slot connections.
    connect( read_topic_editor, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
    connect(btn_proceed, SIGNAL(clicked(bool)),this,SLOT(proceedClicked(bool)));

    display=0;
    d=new CalibPanelPrivate();
    d->displayMagnetometer.showData=false;
    d->displayMagnetometer.scale=1.0;
    d->displayAccelerometer.showData=false;
    d->displayAccelerometer.scale=1.0;
    d->displayGyroscope.showData=false;
    d->displayGyroscope.scale=1.0;
    d->recordData=false;
    d->CalibState = CalibPanelPrivate::Idle;
    d->PreviousState = CalibPanelPrivate::Idle;
    acc_cal=new Sensor3DCalibration(PATH_TO_CALIBRATION, "acc");
    ang_cal=new Sensor3DCalibration(PATH_TO_CALIBRATION, "ang");
    mag_cal=new Sensor3DCalibration(PATH_TO_CALIBRATION, "mag");

    cal_gen=new CalibrationGenerator(
        /*   Magnetometer zero position amplitude: */ 1.00, /* normalized */
        /* Accelerometer zero position  amplitude: */ 9.81, /* m/s^2 */
        /*          Gyro zero position amplitude: */ 1.00);/* normalized */

}

//------------------------------------------------------------------------------------//
CalibPanel::~CalibPanel() {
    delete cal_gen;
    delete mag_cal;
    delete ang_cal;
    delete acc_cal;
    delete d;
}

//------------------------------------------------------------------------------------//

void CalibPanel::imuDataArrived(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::MagneticField::ConstPtr& msg_mag) {
    arma::vec acc_vec(3,arma::fill::zeros); /**< Vector representation of one accelerometer reading */
    arma::vec ang_vec(3,arma::fill::zeros); /**< Vector representation of one gyro reading */
    arma::vec mag_vec(3,arma::fill::zeros); /**< Vector representation of one magnetometer reading */
    ImuData data;
    double scale=1;

    //Convert the ros vector3 datatypes to a armadillo vector (the internally used math library)
    acc_vec(0) = msg_imu->linear_acceleration.x;
    acc_vec(1) = msg_imu->linear_acceleration.y;
    acc_vec(2) = msg_imu->linear_acceleration.z;
    ang_vec(0) = msg_imu->angular_velocity.x;
    ang_vec(1) = msg_imu->angular_velocity.y;
    ang_vec(2) = msg_imu->angular_velocity.z;
    mag_vec(0) = msg_mag->magnetic_field.x;
    mag_vec(1) = msg_mag->magnetic_field.y;
    mag_vec(2) = msg_mag->magnetic_field.z;

    if(display!=0) {
        if(d->displayMagnetometer.showData) {
            scale=d->displayMagnetometer.scale;
            display->DrawPoint(mag_vec(0)/scale,mag_vec(1)/scale,mag_vec(2)/scale,d->displayMagnetometer.color);
        }
        if(d->displayAccelerometer.showData) {
            scale=d->displayAccelerometer.scale;
            display->DrawPoint(acc_vec(0)/scale,acc_vec(1)/scale,acc_vec(2)/scale,d->displayAccelerometer.color);
        }
        if(d->displayGyroscope.showData) {
            scale=d->displayGyroscope.scale;
            display->DrawPoint(ang_vec(0)/scale,ang_vec(1)/scale,ang_vec(2)/scale,d->displayGyroscope.color);
        }
    }
    data.Magnetometer=mag_vec;
    data.Accelerometer=acc_vec;
    data.Gyroscope=ang_vec;

    if(d->recordData) {
        d->currentDataset.push_back(data);
    }

}

//------------------------------------------------------------------------------------//

void CalibPanel::updateTopic() {
    setTopic( read_topic_editor->text() );
}

//------------------------------------------------------------------------------------//

void CalibPanel::proceedClicked(bool checked) {
    static arma::vec zp,zn,xp,xn,yp,yn;

    switch(d->CalibState) {
    case CalibPanelPrivate::Idle:
        switch(d->PreviousState) {
        case CalibPanelPrivate::Idle:
            d->CalibState=CalibPanelPrivate::pZPlane;
            d->displayMagnetometer.showData=false;
            d->displayMagnetometer.color = rviz::Color(1.0,0.0,0.0);
            d->displayAccelerometer.showData=true;
            d->displayAccelerometer.color = rviz::Color(0.5,0.5,0.0);
            break;
        case CalibPanelPrivate::pZPlane:
            d->CalibState=CalibPanelPrivate::nZPlane;
            d->displayMagnetometer.showData=false;
            d->displayMagnetometer.color = rviz::Color(0.5,0.5,0.0);
            d->displayAccelerometer.showData=true;
            d->displayAccelerometer.color = rviz::Color(0.0,0.5,0.5);
            break;
        case CalibPanelPrivate::nZPlane:
            d->CalibState=CalibPanelPrivate::pXPlane;
            d->displayMagnetometer.showData=false;
            d->displayMagnetometer.color = rviz::Color(0.25,0.75,0.0);
            d->displayAccelerometer.showData=true;
            d->displayAccelerometer.color = rviz::Color(0.0,0.75,0.25);
            break;
        case CalibPanelPrivate::pXPlane:
            d->CalibState=CalibPanelPrivate::nXPlane;
            d->displayMagnetometer.showData=false;
            d->displayMagnetometer.color = rviz::Color(0.75,0.25,0.0);
            d->displayAccelerometer.showData=true;
            d->displayAccelerometer.color = rviz::Color(0.0,0.25,0.75);
            break;
        case CalibPanelPrivate::nXPlane:
            d->CalibState=CalibPanelPrivate::pYPlane;
            d->displayMagnetometer.showData=false;
            d->displayMagnetometer.color = rviz::Color(0.5,0.0,0.5);
            d->displayAccelerometer.showData=true;
            d->displayAccelerometer.color = rviz::Color(1.0,1.0,0.0);
            break;
        case CalibPanelPrivate::pYPlane:
            d->CalibState=CalibPanelPrivate::nYPlane;
            d->displayMagnetometer.showData=false;
            d->displayMagnetometer.color = rviz::Color(0.25,0.0,0.75);
            d->displayAccelerometer.showData=true;
            d->displayAccelerometer.color = rviz::Color(0.0,1.0,1.0);
            break;
        case CalibPanelPrivate::nYPlane:
            d->CalibState=CalibPanelPrivate::Sphere;
            d->displayMagnetometer.showData=true;
            d->displayMagnetometer.color = rviz::Color(1.0,0.0,0.0);
            d->displayAccelerometer.showData=false;
            break;
        case CalibPanelPrivate::Sphere:
            d->CalibState=CalibPanelPrivate::Idle;
            display->Clear();
            break;
        }
        btn_proceed->setText("Finish Step");
        d->PreviousState=CalibPanelPrivate::Idle;

        d->displayMagnetometer.scale=2300;

        d->displayAccelerometer.scale=2300;

        d->currentDataset.clear();
        d->recordData=true;
        break;
    case CalibPanelPrivate::pZPlane:
        d->PreviousState=d->CalibState;
        d->CalibState=CalibPanelPrivate::Idle;
        btn_proceed->setText("Record negative Z-Plane");
        d->recordData=false;
        d->displayMagnetometer.showData=false;
        d->displayAccelerometer.showData=false;
        d->displayGyroscope.showData=false;
        zp=cal_gen->GetAccVector(d->currentDataset);

        break;
    case CalibPanelPrivate::nZPlane:
        d->PreviousState=d->CalibState;
        d->CalibState=CalibPanelPrivate::Idle;
        btn_proceed->setText("Record positive X-Plane");
        d->recordData=false;
        d->displayMagnetometer.showData=false;
        d->displayAccelerometer.showData=false;
        d->displayGyroscope.showData=false;
        zn=cal_gen->GetAccVector(d->currentDataset);
        break;
    case CalibPanelPrivate::pXPlane:
        d->PreviousState=d->CalibState;
        d->CalibState=CalibPanelPrivate::Idle;
        btn_proceed->setText("Record negative X-Plane");
        d->recordData=false;
        d->displayMagnetometer.showData=false;
        d->displayAccelerometer.showData=false;
        d->displayGyroscope.showData=false;
        xp=cal_gen->GetAccVector(d->currentDataset);
        break;
    case CalibPanelPrivate::nXPlane:
        d->PreviousState=d->CalibState;
        d->CalibState=CalibPanelPrivate::Idle;
        btn_proceed->setText("Record positive Y-Plane");
        d->recordData=false;
        d->displayMagnetometer.showData=false;
        d->displayAccelerometer.showData=false;
        d->displayGyroscope.showData=false;
        xn=cal_gen->GetAccVector(d->currentDataset);
        break;
    case CalibPanelPrivate::pYPlane:
        d->PreviousState=d->CalibState;
        d->CalibState=CalibPanelPrivate::Idle;
        btn_proceed->setText("Record negative Y-Plane");
        d->recordData=false;
        d->displayMagnetometer.showData=false;
        d->displayAccelerometer.showData=false;
        d->displayGyroscope.showData=false;
        yp=cal_gen->GetAccVector(d->currentDataset);
        break;
    case CalibPanelPrivate::nYPlane:
        d->PreviousState=d->CalibState;
        d->CalibState=CalibPanelPrivate::Idle;
        btn_proceed->setText("Record Sphere");
        d->recordData=false;
        d->displayMagnetometer.showData=false;
        d->displayAccelerometer.showData=false;
        d->displayGyroscope.showData=false;
        yn=cal_gen->GetAccVector(d->currentDataset);
        break;
    case CalibPanelPrivate::Sphere:
        d->PreviousState=d->CalibState;
        d->CalibState=CalibPanelPrivate::Idle;
        btn_proceed->setText("Record positive Z-Plane");
        d->recordData=false;
        d->displayMagnetometer.showData=false;
        d->displayAccelerometer.showData=false;
        d->displayGyroscope.showData=false;
        cal_gen->CalculateMagnetometerCalibrationData(d->currentDataset);
        cal_gen->CalculateAccelerometerCalibrationData(zp,zn,xp,xn,yp,yn);

        cal_gen->InitialiseCalibrationObjectAcc(*acc_cal);
        cal_gen->InitialiseCalibrationObjectMag(*mag_cal);
        {
            arma::vec temp(3,arma::fill::zeros);
            temp(0)=6548.0890872094081;
            temp(1)=6548.0890872094081;
            temp(2)=6548.0890872094081;
            ang_cal->SetSensitivityValues(temp);
        }
        arma::vec calib;
        d->displayMagnetometer.color = rviz::Color(0.0,0.0,1.0);
        for(size_t i=0; i<d->currentDataset.size(); i++) {

            calib=(*mag_cal)(d->currentDataset[i].Magnetometer);
            display->DrawPoint(calib(0),calib(1),calib(2),d->displayMagnetometer.color);
        }
        QMessageBox msgBox;
        msgBox.setText("The imu is calibrated calibration data is found in the current working directory (./calibration) ");
        msgBox.exec();
        break;
    }
}

//------------------------------------------------------------------------------------//

void CalibPanel::setTopic( const QString& new_topic ) {

    if( new_topic != read_topic ) {
        read_topic = new_topic;
        if( read_topic == "" ) {
            read_topic="imu";
        }
        subscriber_.reset(new SubscriberWrapper(nh,read_topic.toStdString()));
        subscriber_->GetSynchronizer()->registerCallback(&CalibPanel::imuDataArrived,(CalibPanel*)this);
        Q_EMIT configChanged();
    }

}

//------------------------------------------------------------------------------------//

void CalibPanel::onInitialize() {
    //Create a display for displaying the magnetometer values.
    display=(CalibDisplay*)vis_manager_->createDisplay("calib_imu/calib_imu_visualization","CalibDisplay",true);
    //Initialize ros node
    nh.setCallbackQueue(vis_manager_->getUpdateQueue ());
    subscriber_.reset(new SubscriberWrapper(nh,"imu"));
    subscriber_->GetSynchronizer()->registerCallback(&CalibPanel::imuDataArrived,(CalibPanel*)this);
}

//------------------------------------------------------------------------------------//

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void CalibPanel::save( rviz::Config config ) const {
    rviz::Panel::save( config );
    config.mapSetValue( "Topic", read_topic );
}

//------------------------------------------------------------------------------------//

// Load all configuration data for this panel from the given Config object.
void CalibPanel::load( const rviz::Config& config ) {
    rviz::Panel::load( config );
    QString topic;
    if( config.mapGetString( "Topic", &topic )) {
        read_topic_editor->setText( topic );
        updateTopic();
    }
}

//------------------------------------------------------------------------------------//

} // end namespace calib_imu

/**************************************************************************************
 * REGISTER RVIZ PLUGIN
 **************************************************************************************/

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(calib_imu::CalibPanel,rviz::Panel )
