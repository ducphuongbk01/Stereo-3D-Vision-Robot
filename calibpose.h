#ifndef CALIBPOSE_H
#define CALIBPOSE_H

#include <iostream>

#include <QDialog>
#include <QDebug>
#include <QThread>
#include <QMessageBox>
#include <QObject>
#include <QTimer>
#include <QTime>
#include <QPlainTextEdit>

#include "calibration.h"
#include "udp.h"
#include "controllib.h"

namespace Ui {
class CalibPose;
}

class CalibPose : public QDialog
{
    Q_OBJECT

public:
    explicit CalibPose(QWidget *parent = nullptr);
    ~CalibPose();

    QString IPAddress;
    quint16 IPPort;
    QPlainTextEdit *QPlText;

public slots:
    void Update_DataCalib();

    void GoHome();

    void CheckRobotConnect();

    void showImage(QImage color, QImage depth);

private slots:
    void on_btnClose_clicked();

    void on_btnBrowser_clicked();

    void on_btnGetPose_clicked();

    void on_btnStartCalib_clicked();

    void on_btnRobotConnect_clicked();

    void on_btnPosX__Pose_clicked();

    void on_btnPosXp_Pose_clicked();

    void on_btnPosY__Pose_clicked();

    void on_btnPosYp_Pose_clicked();

    void on_btnPosZ__Pose_clicked();

    void on_btnPosZp_Pose_clicked();

    void on_btnPosRx__Pose_clicked();

    void on_btnPosRxp_Pose_clicked();

    void on_btnPosRy__Pose_clicked();

    void on_btnPosRyp_Pose_clicked();

    void on_btnPosRzp_Pose_clicked();

    void on_btnPosRz__Pose_clicked();

    void on_btnJointS__Pose_clicked();

    void on_btnJointSp_Pose_clicked();

    void on_btnJointL__Pose_clicked();

    void on_btnJointLp_Pose_clicked();

    void on_btnJointU__Pose_clicked();

    void on_btnJointUp_Pose_clicked();

    void on_btnJointR__Pose_clicked();

    void on_btnJointRp_Pose_clicked();

    void on_btnJointB__Pose_clicked();

    void on_btnJointBp_Pose_clicked();

    void on_btnJointT__Pose_clicked();

    void on_btnJointTp_Pose_clicked();

    void on_btnGoHome_clicked();

    void on_btnForward_clicked();

    void on_Pose_CartTable_cellClicked(int row, int column);

    void on_Pose_JointTable_cellClicked(int row, int column);

    void on_btnCalculate_clicked();

    void on_btnSaveParam_clicked();

private:
    Ui::CalibPose *ui;
    QWidget *parent;
    udp *CalibSocket;
    HandEyeCalibration *calib;
    QTimer *timerTimeout;
};

#endif // CALIBPOSE_H
