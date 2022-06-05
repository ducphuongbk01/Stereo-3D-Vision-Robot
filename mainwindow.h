#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <string>
#include <time.h>

#include <QMainWindow>
#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QMessageBox>
#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QTableWidget>
#include <QSerialPort>
#include <QFileDialog>
#include <QTextBrowser>
#include <QImage>
#include <QPixmap>

#include "udp.h"
#include "calibpose.h"
#include "calibcolor.h"
#include "controllib.h"
#include "serialport.h"
#include "realsensecamera.h"
#include "poseestimation.h"
#include "process.h"
#include "evaluatepose.h"

using namespace std;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void DisplayPulse();
    void DisplayPosition();



public slots:
    void Update_serialport();

    void Home();

    void ADCDisplay(float value);

    void show_lbVideoCapture(QImage img);

    void show_lbVideoCapture_2(QImage img);

    void showPose(int32_t *poses);

    void Sig_poseERROR();

    void Sig_OUTOFWATER();

private slots:
    void Update_Data();

    void SLOT_finishProcess(bool status);


//******************************************//

//              Manual Page                 //
//******************************************//
    void on_btnConnect_clicked();

    void on_btnServo_clicked();

    void on_btnQuit_clicked();

    void on_btnHomePos_clicked();

    void on_btnForwardJoint_clicked();

    void on_btnForwardCart_clicked();

    void on_btnTeach_clicked();

    void on_Man_tableJoint_cellClicked(int row, int column);

    void on_Man_tableCart_cellClicked(int row, int column);

    void on_btnSerialConnect_clicked();

    void on_btnGripper_clicked();

    void on_btnDeleteRow_clicked();

    void on_btnForward_clicked();

    void on_btnStartJob_clicked();

    void on_btnADC_clicked();


    void on_btnJointS__clicked();

    void on_btnJointSp_clicked();

    void on_btnJointLp_clicked();

    void on_btnJointL__clicked();

    void on_btnJointU__clicked();

    void on_btnJointUp_clicked();

    void on_btnJointR__clicked();

    void on_btnJointRp_clicked();

    void on_btnJointB__clicked();

    void on_btnJointBp_clicked();

    void on_btnJointT__clicked();

    void on_btnJointTp_clicked();

    void on_btnPosX__clicked();

    void on_btnPosXp_clicked();

    void on_btnPosY__clicked();

    void on_btnPosYp_clicked();

    void on_btnPosZ__clicked();

    void on_btnPosZp_clicked();

    void on_btnPosRx__clicked();

    void on_btnPosRxp_clicked();

    void on_btnPosRy__clicked();

    void on_btnPosRyp_clicked();

    void on_btnPosRz__clicked();

    void on_btnPosRzp_clicked();

//              Auto Page                 //
//******************************************//
    void on_btnStartProcess_clicked();

    void on_btnCamera_clicked();

    void on_btnSerialConnect_2_clicked();

    void on_btnRobotConnect_clicked();

    void on_btnHome_clicked();

    void on_btnGripper_2_clicked();

    void on_btnADC_2_clicked();

    void on_btnInit_clicked();

//              Config Page                 //
//******************************************//

    void on_Config_btnCameraBrowser_clicked();

    void on_Config_btnPoseForm_clicked();

    void on_Config_btnColorForm_clicked();

    void on_Config_btnPoseBrowser_clicked();

    void on_Config_btnColorBrowser_clicked();

    void on_Config_btnModelBrowser_clicked();

    void on_Config_btnClassBrowser_clicked();

    void on_Config_btnBottleBrowser_clicked();

    void on_Config_btnCupBrowser_clicked();

    void on_btnUpdateRefFrame_clicked();

    void on_btnSavePC_clicked();

    void on_btnBrowserJoint_clicked();

    void on_btnBrowserCart_clicked();

    void on_btnBrowser_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_clicked();

    void on_btnInit_Eva_clicked();

    void on_btnGetPose_clicked();

    void on_led_PE_clicked();

    void on_led_OOW_clicked();

    void on_btnEmer_clicked();

    void on_Config_btnHome_clicked();

private:
    std::string cameraFile, poseCalibFile, colorCalibFile, yoloModelFile, yoloClassFile, bottleModelFile, cupModelFile;
    udp *socket;
    serialPort *stm;
    RealsenseCamera *camera;
    PoseEstimation *poseEstimator;
    process *Waterprocess;

    Ui::MainWindow *ui;
    CalibPose *CalibposeForm;
    CalibColor *CalibColorForm;
    QTimer *timerTimeout;
    QTimer *serialupdateTimer;

    EvaluatePose *eva;

    bool check_file();
    void readfile_mainwindow();
    void UpdatePixmap();
};
#endif // MAINWINDOW_H
