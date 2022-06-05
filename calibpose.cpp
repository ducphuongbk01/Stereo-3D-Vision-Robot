#include "calibpose.h"
#include "ui_calibpose.h"

#include <QTime>
#include <QTimer>
#include <QDir>
#include <QFileDialog>
#include <QPlainTextEdit>
#include <iostream>

std::vector<int32_t> Pose_Pos;

int32_t Pose_Cartes[6], Pose_Joint[6];

int Pose_Mode = 0;

float Pose_teaching_pos[6];

CalibPose::CalibPose(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CalibPose)
{
    ui->setupUi(this);
    setWindowTitle("Calibration Pose Form");
    this->parent = parent;

    this->timerTimeout = new QTimer;

    QPlText = new QPlainTextEdit;

    CheckRobotConnect();

}

CalibPose::~CalibPose()
{
    delete ui;
}

void CalibPose::showImage(QImage color, QImage depth)
{
    ControlLib::ShowImage(this->ui->lbVideoCapture_3, color, 640, 480);
    ControlLib::ShowImage(this->ui->lbVideoCapture_Pose, depth, 640, 480);
}

void CalibPose::Update_DataCalib()
{
    if(this->CalibSocket->readCart == true || this->CalibSocket->readPos == true)
    {
        if(this->CalibSocket->Get_CartPos()->size() == 84)
        {
            for(int i = 0; i< 6;i++)
            {
                Pose_Pos.push_back(this->CalibSocket->ByteArray2Int32(this->CalibSocket->Get_CartPos(),52 + 4*i,4));
            }
            for(uint i = 0; i<Pose_Pos.size();i++)
            {
                Pose_Cartes[i] = Pose_Pos.at(i);
            }
            Pose_Pos.clear();
            ControlLib::DisplayPosition(ui->tbCurrentPosX_3, ui->tbCurrentPosY_3, ui->tbCurrentPosZ_3,
                                  ui->tbCurrentPosRoll_3, ui->tbCurrentPosPitch_3, ui->tbCurrentPosYaw_3, Pose_Cartes);
        }
    }
    if(this->CalibSocket->readJoint == true || this->CalibSocket->readPos == true)
    {
        if(this->CalibSocket->Get_PulsePos()->size() == 76)
        {
            for(int i = 0; i< 6;i++)
            {
                Pose_Pos.push_back(this->CalibSocket->ByteArray2Int32(this->CalibSocket->Get_PulsePos(),52 + 4*i,4));
            }
            for(uint i = 0; i<Pose_Pos.size();i++)
            {
                Pose_Joint[i] = Pose_Pos.at(i);
            }
            Pose_Pos.clear();
            ControlLib::DisplayJoint(ui->tbCurrentS, ui->tbCurrentL, ui->tbCurrentU,
                                     ui->tbCurrentR, ui->tbCurrentB, ui->tbCurrentT, Pose_Joint);
        }
    }
}

void CalibPose::GoHome()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    bool state = this->CalibSocket->WritePosJoint(this->ui->tbCartSpeed_Pose->toPlainText().toInt(),0,0,0,0,0,0);
    ControlLib::delay_ms(1000);
    this->timerTimeout->start(1000);

    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Can not go home position.");
    }
    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}

void CalibPose::CheckRobotConnect()
{
    bool flag = ui->btnRobotConnect->text()=="Robot Connect";

    this->ui->btnGetPose->setEnabled(!flag);
    this->ui->btnForward->setEnabled(!flag);
    this->ui->btnGoHome->setEnabled(!flag);

    this->ui->btnJointSp_Pose->setEnabled(!flag);
    this->ui->btnJointLp_Pose->setEnabled(!flag);
    this->ui->btnJointUp_Pose->setEnabled(!flag);
    this->ui->btnJointRp_Pose->setEnabled(!flag);
    this->ui->btnJointBp_Pose->setEnabled(!flag);
    this->ui->btnJointTp_Pose->setEnabled(!flag);

    this->ui->btnJointS__Pose->setEnabled(!flag);
    this->ui->btnJointL__Pose->setEnabled(!flag);
    this->ui->btnJointU__Pose->setEnabled(!flag);
    this->ui->btnJointR__Pose->setEnabled(!flag);
    this->ui->btnJointB__Pose->setEnabled(!flag);
    this->ui->btnJointT__Pose->setEnabled(!flag);

    this->ui->btnPosXp_Pose->setEnabled(!flag);
    this->ui->btnPosYp_Pose->setEnabled(!flag);
    this->ui->btnPosZp_Pose->setEnabled(!flag);
    this->ui->btnPosRxp_Pose->setEnabled(!flag);
    this->ui->btnPosRyp_Pose->setEnabled(!flag);
    this->ui->btnPosRzp_Pose->setEnabled(!flag);

    this->ui->btnPosX__Pose->setEnabled(!flag);
    this->ui->btnPosY__Pose->setEnabled(!flag);
    this->ui->btnPosZ__Pose->setEnabled(!flag);
    this->ui->btnPosRx__Pose->setEnabled(!flag);
    this->ui->btnPosRy__Pose->setEnabled(!flag);
    this->ui->btnPosRz__Pose->setEnabled(!flag);

}

void CalibPose::on_btnRobotConnect_clicked()
{
    if(ui->btnRobotConnect->text()=="Robot Connect")
    {
        QHostAddress h;
        h.setAddress(IPAddress);
        quint16 p = IPPort;
        CalibSocket = new udp(h,p);

        calib = new HandEyeCalibration( "/home/phuongdoan/Code/ThesisGUI/config.json", CalibSocket);
        connect(this->calib, SIGNAL(framesReady(QImage,QImage)), this, SLOT(showImage(QImage,QImage)));

        bool state = this->CalibSocket->ConnectMotoman();
        ControlLib::delay_ms(200);
        this->CalibSocket->start();
        Pose_Mode = 0;
        ControlLib::Update_Pos(Pose_Mode,true, CalibSocket);
        // Lay Signal tu Thread de doc vi tri cua Robot
        connect(this->timerTimeout, SIGNAL(timeout()), this->CalibSocket, SLOT(OFFTrigger()));

        ControlLib::Update_Pos(Pose_Mode,false, CalibSocket);
        ControlLib::delay_ms(1000);

        bool state1 = this->CalibSocket->OnServo();
        ControlLib::delay_ms(2000);
        this->timerTimeout->start(1000);

        if(state1)
        {
            this->timerTimeout->stop();
            this->CalibSocket->start();
            Pose_Mode = 0;
            ui->btnRobotConnect->setText("Robot Disconnect");
            connect(this->CalibSocket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_DataCalib()));
            ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
            CheckRobotConnect();

        }
        else
        {
            disconnect(this->CalibSocket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_DataCalib()));
            disconnect(this->timerTimeout, SIGNAL(timeout()), this->CalibSocket, SLOT(OFFTrigger()));
            this->CalibSocket->Stopthread();
            QMessageBox::warning(this, "Warning", "Can not connect to Robot");
            ui->btnRobotConnect->setText("Robot Connect");
            CheckRobotConnect();

        }
    }
    else
    {
        ControlLib::Update_Pos(Pose_Mode, false, this->CalibSocket);
        ControlLib::delay_ms(10);
        disconnect(this->CalibSocket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_DataCalib()));
        this->CalibSocket->Stopthread();
        bool state1 = this->CalibSocket->OffServo();
        ControlLib::delay_ms(1000);
        bool state = this->CalibSocket->DisconnectMotoman();
        ControlLib::delay_ms(200);
        if(state1)
        {
            ui->btnRobotConnect->setText("Robot Connect");
            disconnect(this->timerTimeout, SIGNAL(timeout()), this->CalibSocket, SLOT(OFFTrigger()));
            
        }
        else
        {
            connect(this->CalibSocket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_DataCalib()));
            QMessageBox::warning(this,"Warning!!!","Cant connect");
        }
        CheckRobotConnect();
    }
}


void CalibPose::on_btnGoHome_clicked()
{
    this->ui->btnGoHome->setEnabled(false);
    this->GoHome();
    ControlLib::delay_ms(1000);
    this->ui->btnGoHome->setEnabled(true);
}

void CalibPose::on_btnStartCalib_clicked()
{
    if (this->ui->btnStartCalib->text() == "Start Calibrate")
    {
        if(this->ui->btnRobotConnect->text() == "Robot Disconnect")
        {
            this->calib->start();
            this->ui->btnStartCalib->setText("Stop Calibrate");
        }
        else
        {
            QMessageBox::warning(this,"Warning!!!","Please connect to robot first");
        }
    }
    else
    {
        this->calib->stop();
        this->ui->btnStartCalib->setText("Start Calibrate");
    }

}

void CalibPose::on_btnClose_clicked()
{
    if(this->ui->btnRobotConnect->text() == "Robot Disconnect")
    {
        QMessageBox::warning(this, "Warning", "Please Disconnect to Robot");
    }
    else if(this->ui->btnStartCalib->text() == "Stop Calibrate")
    {
        QMessageBox::warning(this, "Warning", "Please Stop Calibrate");
    }
    else
    {
        delete this->calib;
        this->close();
        this->parent->show();
    }
}


void CalibPose::on_btnBrowser_clicked()
{
    ui->tbCalibPoseURL->setText(QFileDialog::getExistingDirectory(this, "Choose directory", QDir::homePath()));
}


void CalibPose::on_btnGetPose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode, false, this->CalibSocket);
    ControlLib::delay_ms(10);
    if(ui->btnRobotConnect->text() == "Robot Connect")
    {
       QMessageBox::information(this,"Warning!!!","Pleased Connect to Robot");
    }
    else
    {
        this->calib->startTrigger(Pose_Joint);
        if(this->ui->CalibPose_Tab->currentWidget() == this->ui->JointTab)
        {
            if(this->CalibSocket->Get_PulsePos()->size() == 76)
            {
                ControlLib::Teaching(ui->Pose_JointTable,ui->tbCurrentS, ui->tbCurrentL, ui->tbCurrentU,
                                     ui->tbCurrentR, ui->tbCurrentB, ui->tbCurrentT);
            }
            else
            {
                QMessageBox::warning(this,"Error","Cant get Joint Position");
            }


        }
        if(this->ui->CalibPose_Tab->currentWidget() == this->ui->CartTab)
        {
            if(this->CalibSocket->Get_CartPos()->size() == 84)
            {
                ControlLib::Teaching(ui->Pose_CartTable,ui->tbCurrentPosX_3, ui->tbCurrentPosY_3, ui->tbCurrentPosZ_3,
                                     ui->tbCurrentPosRoll_3, ui->tbCurrentPosPitch_3, ui->tbCurrentPosYaw_3);
            }
            else
            {
                QMessageBox::warning(this,"Error","Cant get Cart Position");
            }
        }
    }
    ControlLib::Update_Pos(Pose_Mode, true, this->CalibSocket);
}

void CalibPose::on_btnCalculate_clicked()
{
    this->calib->caculatePose();
}


void CalibPose::on_btnSaveParam_clicked()
{
    Convert::saveMatFile(calib->w2c_transmit,ui->tbCalibPoseURL->toPlainText().toStdString()+"/calibw2c.yml");
    Convert::saveMatFile(calib->o2e_transmit,ui->tbCalibPoseURL->toPlainText().toStdString()+"/calibo2e.yml");
}

void CalibPose::on_btnForward_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    if(ui->btnRobotConnect->text() == "Robot Connect")
    {
       QMessageBox::information(this,"Warning!!!","Pleased Connect to Robot");
    }
    else
    {
        if(ui->CalibPose_Tab->currentWidget() == ui->CartTab)
        {
            bool state = this->CalibSocket->WritePosCart(this->ui->tbCartSpeed_Pose->toPlainText().toInt(),Pose_teaching_pos[0],Pose_teaching_pos[1],Pose_teaching_pos[2],
                    Pose_teaching_pos[3],Pose_teaching_pos[4],Pose_teaching_pos[5]);
            ControlLib::delay_ms(50);
            this->timerTimeout->start(1000);
            if(state)
            {
                this->timerTimeout->stop();
            }
            else
            {
                this->timerTimeout->stop();
                QMessageBox::warning(this, "Warining", "Cannot connect.");
            }
        }
        if(this->ui->CalibPose_Tab->currentWidget() == this->ui->JointTab)
        {
            bool state = this->CalibSocket->WritePosJoint(this->ui->tbCartSpeed_Pose->toPlainText().toInt(),Pose_teaching_pos[0],Pose_teaching_pos[1],Pose_teaching_pos[2],
                    Pose_teaching_pos[3],Pose_teaching_pos[4],Pose_teaching_pos[5]);
            ControlLib::delay_ms(50);
            this->timerTimeout->start(1000);
            if(state)
            {
                this->timerTimeout->stop();
            }
            else
            {
                this->timerTimeout->stop();
                QMessageBox::warning(this, "Warining", "Cannot connect.");
            }
        }
    }
    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}

void CalibPose::on_Pose_CartTable_cellClicked(int row, int column)
{
    bool ok;
    ok = ui->Pose_CartTable->item(row, 0)->text().size() == 0;
    if(!ok)
    {
        for(int i = 0; i < 6; i++)
        {
            Pose_teaching_pos[i] = ui->Pose_CartTable->item(row, i)->text().toFloat();
//            qDebug() << Pose_teaching_pos[i];
        }
    }
    else
    {
        qDebug() << "No value";
    }
}


void CalibPose::on_Pose_JointTable_cellClicked(int row, int column)
{
    bool ok;
    ok = ui->Pose_JointTable->item(row, 0)->text().size() == 0;
    if(!ok)
    {
        for(int i = 0; i < 6; i++)
        {
            Pose_teaching_pos[i] = ui->Pose_JointTable->item(row, i)->text().toFloat();
            // qDebug() << Pose_teaching_pos[i];
        }
    }
    else
    {
        qDebug() << "No value";
    }
}


void CalibPose::on_btnPosX__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(1,-1, var, CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosXp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(1,+1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosY__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(2,-1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosYp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(2,+1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosZ__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(3,-1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosZp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(3,+1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosRx__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(4,-1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosRxp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(4,+1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosRy__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(5,-1, var, this->CalibSocket);
    this->timerTimeout->start(1000);       
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }


    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosRyp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(5,+1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnPosRz__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(6,-1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}

void CalibPose::on_btnPosRzp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);

    float speed = ui->tbCartSpeed_Pose->toPlainText().toDouble();
    float X = ui->tbCurrentPosX_3->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY_3->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ_3->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll_3->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch_3->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw_3->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(6,+1, var, this->CalibSocket);
    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }
    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointS__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(1,-1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointSp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(1,+1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointL__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(2,-1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointLp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(2,+1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointU__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(3,-1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointUp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(3,+1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointR__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(4,-1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointRp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(4,+1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointB__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(5,-1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointBp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(5,+1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointT__Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(6,-1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}


void CalibPose::on_btnJointTp_Pose_clicked()
{
    ControlLib::Update_Pos(Pose_Mode,false,this->CalibSocket);
    float speed = ui->tbJointSpeed_Pose->toPlainText().toDouble();
    float R1 = ui->tbCurrentS->toPlainText().toDouble();
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(6,+1, var, this->CalibSocket);

    std::vector<float> joints_limit = {var[1],var[2],var[3],var[4],var[5],var[6]};

    bool WS = Convert::JointWorkspace(joints_limit);

    if(!WS)
    {
        QMessageBox::warning(this, "Warining", "Out of WorkSpace");

    }

    this->timerTimeout->start(1000);
    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        if(WS)
        {
            QMessageBox::warning(this, "Warining", "Cannot connect.");
        }
    }

    ControlLib::Update_Pos(Pose_Mode,true,this->CalibSocket);
}
