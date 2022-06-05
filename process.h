#ifndef PROCESS_H
#define PROCESS_H

#include <QThread>
#include <iostream>
#include <QSerialPort>
#include <QPlainTextEdit>
#include <QTimer>
#include <QString>
#include <QMetaType>
#include "udp.h"
#include "serialport.h"
#include "controllib.h"
#include "poseestimation.h"
#include "realsensecamera.h"

class process  : public QThread
{
    Q_OBJECT
public:
    process(int Mode, udp *CtrlSocket, serialPort *Stm, RealsenseCamera *camera,
            std::string calib_path, std::string bottle_path, std::string cup_path);
    void run();

    void startThread();
    void stopThread();


    // Varaiable
    int processMode;

    bool bStart=false;

    bool bStartProcess=false;

    bool bGapChai=false;
    bool bTraChai=false;
    bool bGapLy=false;
    bool bFinishGapLy = false;
    bool bTraLy=false;
    bool bRotnuoc=false;
    bool bFullWater= false;
    bool bClear=false;
    bool bMain=false;

    float lvl;
    float full_water;

    udp *processSocket;
    serialPort *processStm;
    QTimer *timerlvl;
    QPlainTextEdit *ProcessTxt;

public slots:
    void Waterlvl(float level);
    void IpruptTimer();

    void JOB_GapChai();
    void JOB_GapLy();
    void JOB_TraChai();
    void JOB_TraLy();
    void JOB_Rotnuoc();
    void JOB_Domucnuoc();
    void JOB_Clear(int mode);
    void JOB_Main();

    void Startprocess();
    void caculate();


signals:
    void Sig_JOBGapChai();

    void Sig_JOBTraChai();

    void Sig_JOBGapLy();

    void Sig_JOBTraLy();

    void Sig_JOBRotnuoc();

    void Sig_JOBDomucnuoc();

    void Sig_JOBClear(int mode);

    void Sig_JOBMain();

    void Sig_finishProcess(bool sig);

    void finish(int32_t *pulseWrite);

    void Sig_poseERROR();

    void Sig_OOW();

private:
    cv::Matx44d c2b, g2e;
    RealsenseCamera *camera;
    PoseEstimation *ppf;
};

#endif // PROCESS_H
