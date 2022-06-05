#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QObject>
#include <QTableWidget>
#include <QTime>
#include <QTimer>
#include <QtCore>
#include <QtGui>
#include <QElapsedTimer>
#include <QtSerialPort>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QPixmap>
#include <QPalette>
#include <QColor>
#include <opencv2/opencv.hpp>

#include "controllib.h"
#include "serialport.h"
#include "process.h"

#define BUFF_SIZE 11


std::vector<int32_t> Pos;

int32_t cartes[6], joint[6];
float teaching_pos[6];

int Mode = 0;

// lvl
bool bEmer = false;
bool bOoWater = false;

int32_t Joint_R1, Joint_R2, Joint_R3, Joint_R4, Joint_R5, Joint_R6, Joint_Speed;
int32_t Cart_X, Cart_Y, Cart_Z, Cart_Roll, Cart_Pitch, Cart_Yaw, Cart_Speed;

QList<QSerialPortInfo> list;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Robot Communication");

    UpdatePixmap();

    socket = NULL;
    Waterprocess = NULL;
    camera = NULL;


    CalibposeForm = new CalibPose(this);
    CalibColorForm = new CalibColor(this);

//    serialPort stm("ttyUSB0",115200);

    this->timerTimeout = new QTimer;
    this->serialupdateTimer = new QTimer;
    this->serialupdateTimer->start(1000);
    connect(this->serialupdateTimer, SIGNAL(timeout()), this, SLOT(Update_serialport()));

    this->ui->btnCamera->setEnabled(false);
    this->ui->btnUpdateRefFrame->setEnabled(false);
    this->ui->btnHome->setEnabled(false);
    this->ui->btnGripper_2->setEnabled(false);
    this->ui->btnADC_2->setEnabled(false);
    this->ui->btnSavePC->setEnabled(false);
    this->ui->btnStartProcess->setEnabled(false);
}


MainWindow::~MainWindow()
{
    disconnect(this->serialupdateTimer, SIGNAL(timeout()), this, SLOT(Update_serialport()));

    if(Waterprocess != NULL)
    {
        Waterprocess->stopThread();
        if(!Waterprocess->wait(5000))
        {
            Waterprocess->terminate();
            Waterprocess->wait();
        }
    }

    if(camera != NULL)
    {
        camera->stop();
        if(!camera->wait(5000))
        {
            camera->terminate();
            camera->wait();
        }
    }

    if(socket != NULL)
    {
        socket->Stopthread();
        if(!socket->wait(5000))
        {
            socket->terminate();
            socket->wait();
        }
    }

    ControlLib::delay_ms(500);

    delete Waterprocess;
    delete camera;
//    delete socket;

    delete ui;
}

void MainWindow::UpdatePixmap()
{
    QPixmap pix("/home/phuongdoan/Code/ThesisGUI/LogoBK.png");
    QPixmap pix1("/home/phuongdoan/Code/ThesisGUI/logoYas.png");
    QPixmap pix2("/home/phuongdoan/Code/ThesisGUI/computerlogo.png");

    this->ui->lbVideoCapture->setPixmap(pix2.scaled(640,480,Qt::KeepAspectRatio));
    this->ui->lbVideoCapture_2->setPixmap(pix2.scaled(640,480,Qt::KeepAspectRatio));
    this->ui->label_6->setPixmap(pix.scaled(331,301,Qt::KeepAspectRatio));
    this->ui->label_5->setPixmap(pix1.scaled(1331,261,Qt::KeepAspectRatio));
}

bool MainWindow::check_file()
{
    if(this->ui->Config_tbCamerafile->toPlainText() == "" ||
       this->ui->Config_tbPosefile->toPlainText() == "" ||
       this->ui->Config_tbColorfile->toPlainText() == "" ||
       this->ui->Config_tbModelfile->toPlainText() == "" ||
       this->ui->Config_tbClassfile->toPlainText() == "" ||
       this->ui->Config_tbBottlefile->toPlainText() == "" ||
       this->ui->Config_tbCupfile->toPlainText() == "")
    {
        return false;
    }
    else
    {
        return true;
    }
}

void MainWindow::readfile_mainwindow()
{
    this->cameraFile = this->ui->Config_tbCamerafile->toPlainText().toStdString();
    this->poseCalibFile = this->ui->Config_tbPosefile->toPlainText().toStdString();
    this->colorCalibFile = this->ui->Config_tbColorfile->toPlainText().toStdString();
    this->yoloModelFile = this->ui->Config_tbModelfile->toPlainText().toStdString();
    this->yoloClassFile = this->ui->Config_tbClassfile->toPlainText().toStdString();
    this->bottleModelFile = this->ui->Config_tbBottlefile->toPlainText().toStdString();
    this->cupModelFile = this->ui->Config_tbCupfile->toPlainText().toStdString();
}

void MainWindow::Update_serialport()
{
    ui->cbSerialCom->clear();
    ui->cbSerialCom_2->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        if(info.portName() != "ttyACM0")
        {
           ui->cbSerialCom->addItem(info.portName());
           ui->cbSerialCom_2->addItem(info.portName());
        }
    }
}

void MainWindow::Home()
{
    bool state = ControlLib::GoHome(0,this->socket);

    if(state)
    {
        this->timerTimeout->stop();
    }
    else
    {
        this->timerTimeout->stop();
        QMessageBox::warning(this, "Warining", "Cannot connect.");
    }
//    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::Update_Data()
{
    if(this->socket->readCart == true || this->socket->readPos == true)
    {
        if(this->socket->Get_CartPos()->size() == 84)
        {
            for(int i = 0; i< 6;i++)
            {
                Pos.push_back(this->socket->ByteArray2Int32(this->socket->Get_CartPos(),52 + 4*i,4));
            }
            for(uint i = 0; i<Pos.size();i++)
            {
                cartes[i] = Pos.at(i);
            }
            Pos.clear();
            DisplayPosition();
        }
    }
    if(this->socket->readJoint == true || this->socket->readPos == true)
    {
        if(this->socket->Get_PulsePos()->size() == 76)
        {
            for(int i = 0; i< 6;i++)
            {
                Pos.push_back(this->socket->ByteArray2Int32(this->socket->Get_PulsePos(),52 + 4*i,4));
            }
            for(uint i = 0; i<Pos.size();i++)
            {
                joint[i] = Pos.at(i);
            }
            Pos.clear();
            DisplayPulse();
        }
    }
}


void MainWindow::DisplayPosition()
{
    ControlLib::DisplayPosition(ui->tbCurrentPosX, ui->tbCurrentPosY, ui->tbCurrentPosZ,
                          ui->tbCurrentPosRoll, ui->tbCurrentPosPitch, ui->tbCurrentPosYaw, cartes);

    ControlLib::DisplayPosition(ui->tbCurrentPosX_2, ui->tbCurrentPosY_2, ui->tbCurrentPosZ_2,
                          ui->tbCurrentPosRoll_2, ui->tbCurrentPosPitch_2, ui->tbCurrentPosYaw_2, cartes);
}


void MainWindow::DisplayPulse()
{
    ControlLib::DisplayJoint(ui->tbCurrentS, ui->tbCurrentL, ui->tbCurrentU,
                             ui->tbCurrentR, ui->tbCurrentB, ui->tbCurrentT, joint);

    ControlLib::DisplayJoint(ui->tbCurrentS_2, ui->tbCurrentL_2, ui->tbCurrentU_2,
                             ui->tbCurrentR_2, ui->tbCurrentB_2, ui->tbCurrentT_2, joint);
}
//******************************************//

//******************************************//


//              Manual Page                 //
//******************************************//

//              Robot Connection            //
//******************************************//



void MainWindow::on_btnQuit_clicked()
{
    disconnect(this->socket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_Data()));
    disconnect(this->timerTimeout, SIGNAL(timeout()), this->socket, SLOT(OFFTrigger()));

    if(this->ui->btnServo->text() == "Servo OFF")
    {
        ControlLib::Update_Pos(Mode,false,this->socket);
        ControlLib::delay_ms(50);
        bool state = this->socket->OffServo();
        this->timerTimeout->start(1000);
        ControlLib::delay_ms(200);
        if(state)
        {
            ControlLib::Update_Pos(Mode,true,this->socket);
            this->ui->btnServo->setText("Servo ON");
            this->timerTimeout->stop();
        }
        else
        {
            ControlLib::Update_Pos(Mode,true,this->socket);
            QMessageBox::warning(this, "Warning", "Cant Connect");
            this->timerTimeout->stop();
        }
    }

    if(this->ui->btnConnect->text() == "Disconnect")
    {
        ControlLib::Update_Pos(Mode,false,socket);
        ControlLib::delay_ms(10);
        this->socket->Stopthread();
        bool state = this->socket->DisconnectMotoman();
        ControlLib::delay_ms(200);
        if(state)
        {
            this->ui->btnConnect->setText("Connect");
            disconnect(this->timerTimeout, SIGNAL(timeout()), this->socket, SLOT(OFFTrigger()));
        }
        else
        {
            QMessageBox::warning(this,"Warning!!!","Cant connect");
        }
    }

    if(this->socket != NULL) this->socket->terminate();

    QApplication::quit();
}
//******************************************//

//              Robot Group                 //
//******************************************//
void MainWindow::on_btnConnect_clicked()
{
    this->ui->btnServo->setEnabled(false);
    if(ui->btnConnect->text()=="Connect")
    {
        QHostAddress h;
        h.setAddress(this->ui->tbIPAddress->toPlainText());
        quint16 p = this->ui->tbPort->toPlainText().toUInt();
        socket = new udp(h,p);
        bool state = this->socket->ConnectMotoman();
        ControlLib::delay_ms(200);
        ControlLib::Update_Pos(Mode,false,this->socket);
        int32_t status;
        bool readstatus = this->socket->ReadStatus(&status);
        ControlLib::delay_ms(200);

        if(readstatus)
        {
            ControlLib::Update_Pos(Mode,true,this->socket);
            this->ui->btnConnect->setText("Disconnect");
            this->ui->btnRobotConnect->setText("Robot Disconnect");
            this->ui->btnRobotConnect->setEnabled(false);
            ControlLib::AppOutput("Robot Connected",this->ui->pteOutput);
            this->socket->start();
            Mode = 0;
            ControlLib::Update_Pos(Mode,true,this->socket);
            // Lay Signal tu Thread de doc vi tri cua Robot
            connect(this->socket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_Data()));
            connect(this->timerTimeout, SIGNAL(timeout()), this->socket, SLOT(OFFTrigger()));

            if( status>>6 == 1)
            {
                this->ui->btnServo->setText("Servo OFF");
            }
            else if(status>>6 == 0)
            {
                this->ui->btnServo->setText("Servo ON");
            }
        }
        else
        {
            QMessageBox::warning(this,"Warning!!!","Cant connect");
        }


    }
    else
    {
        ControlLib::Update_Pos(Mode,false,this->socket);
        ControlLib::delay_ms(10);
        this->socket->Stopthread();
        bool state = this->socket->DisconnectMotoman();
        ControlLib::delay_ms(200);
        if(state)
        {
            this->ui->btnConnect->setText("Connect");
            this->ui->btnRobotConnect->setText("Robot Connect");
            this->ui->btnRobotConnect->setEnabled(true);
            ControlLib::AppOutput("Robot Disconnected",this->ui->pteOutput);
            disconnect(this->timerTimeout, SIGNAL(timeout()), this->socket, SLOT(OFFTrigger()));
//            delete socket;
        }
        else
        {
            QMessageBox::warning(this,"Warning!!!","Cant connect");
        }
    }
    ControlLib::delay_ms(1000);
    this->ui->btnServo->setEnabled(true);
}


void MainWindow::on_btnServo_clicked()
{
    if(ui->btnConnect->text() == "Connect")
    {
       QMessageBox::warning(this,"Warning!!!","Pleased Connect UDP first!");
    }
    else
    {
        if(ui->btnServo->text() == "Servo ON")
        {
            ControlLib::Update_Pos(Mode,false,this->socket);
            ControlLib::delay_ms(50);
            bool state = this->socket->OnServo();
            ControlLib::delay_ms(200);
            this->timerTimeout->start(1000);

            ui->btnServo->setText("Servo OFF");
            if(state)
            {
                ControlLib::Update_Pos(Mode,true,this->socket);
                this->timerTimeout->stop();
            }
            else
            {
                ui->btnServo->setText("Servo ON");
                QMessageBox::warning(this, "Warning", "Cant Connect");
                ControlLib::Update_Pos(Mode,true,this->socket);
                this->timerTimeout->stop();
            }
        }
        else
        {
            ControlLib::Update_Pos(Mode,false,this->socket);
            ControlLib::delay_ms(50);
            bool state = this->socket->OffServo();
            this->timerTimeout->start(1000);
            ControlLib::delay_ms(200);
            if(state)
            {
                ControlLib::Update_Pos(Mode,true,this->socket);
                ui->btnServo->setText("Servo ON");
                this->timerTimeout->stop();
            }
            else
            {
                ControlLib::Update_Pos(Mode,true,this->socket);
                QMessageBox::warning(this, "Warning", "Cant Connect");
                this->timerTimeout->stop();
            }
        }
    }
}

void MainWindow::on_btnHomePos_clicked()
{
    if(ui->btnConnect->text() == "Connect")
    {
       QMessageBox::warning(this,"Warning!!!","Pleased Connect UDP first then Turn on Servo");
    }
    else if(ui->btnServo->text() == "Servo ON")
    {
        QMessageBox::warning(this,"Warning!!!","Pleased turn on Servo");
    }
    else
    {
        ControlLib::Update_Pos(Mode,false,this->socket);
        bool state = this->socket->WritePosJoint(10,udp::Home_1[0],udp::Home_1[1],udp::Home_1[2],udp::Home_1[3],udp::Home_1[4],udp::Home_1[5]);
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
        ControlLib::Update_Pos(Mode,true,this->socket);
    }
}

void MainWindow::on_btnStartJob_clicked()
{
    if(ui->btnConnect->text()=="Connect")
    {
       QMessageBox::information(this,"Warning!!!","Pleased Connect UDP first then Turn on Servo");
    }
    else if(ui->btnServo->text() == "Servo ON")
    {
        QMessageBox::information(this,"Warning!!!","Pleased turn on Servo");
    }
    else
    {
        ControlLib::Update_Pos(Mode, false, this->socket);
        QString Job = ui->cbJob->currentText();
        char jobname[Job.length()];
        strcpy(jobname,Job.toStdString().c_str());
        this->socket->SelectJob(jobname);
        ControlLib::delay_ms(500);
        this->socket->StartJob();
        ControlLib::delay_ms(500);


        ControlLib::Update_Pos(Mode, true, this->socket);
    }
}


//              Joint Tab                    //
//******************************************//
void MainWindow::on_btnForwardJoint_clicked()
{
    if(ui->btnConnect->text() == "Connect")
    {
       QMessageBox::information(this,"Warning!!!","Pleased Connect UDP first then Turn on Servo");
    }
    else if(ui->btnServo->text() == "Servo ON")
    {
        QMessageBox::information(this,"Warning!!!","Pleased turn on Servo");
    }
    else
    {
        bool s, l, u, r, b, t, v;
        s = ui->txtJointS->text().size()==0;
        l = ui->txtJointL->text().size()==0;
        u = ui->txtJointU->text().size()==0;
        r = ui->txtJointR->text().size()==0;
        b = ui->txtJointB->text().size()==0;
        t = ui->txtJointT->text().size()==0;
        v = ui->txtJointSpeed->text().size()==0;
        if(s||l||u||r||b||t)
        {
            QMessageBox::critical(this,"Error","Please insert the position!");
        }
        else if (v)
        {
            QMessageBox::critical(this,"Error","Please insert the velocity!");
        }
        else
        {
            float S = ui->txtJointS->text().toDouble(&s);
            float L = ui->txtJointL->text().toDouble(&l);
            float U = ui->txtJointU->text().toDouble(&u);
            float R = ui->txtJointR->text().toDouble(&r);
            float B = ui->txtJointB->text().toDouble(&b);
            float T = ui->txtJointT->text().toDouble(&t);
            float speed = ui->txtJointSpeed->text().toDouble(&v);
            if(s && l && u && r && b && t && v)
            {

                ControlLib::Update_Pos(Mode,false,this->socket);

                if(speed < 0 || speed > 100)
                {
                    QMessageBox::warning(this,"Error","Please check the speed setpoint!");
                }

                std::vector<float> joints_limit = {S,L,U,R,B,T};

                bool WS = Convert::JointWorkspace(joints_limit);
                if(WS)
                {
                    bool state = this->socket->WritePosJoint(speed,S,L,U,R,B,T);
                    ControlLib::delay_ms(100);
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
                    ControlLib::Update_Pos(Mode,true,this->socket);
                }
                else
                {
                  QMessageBox::warning(this,"Error","OUT OF WORKSPACE!");
                }
            }
            else
            {
              QMessageBox::warning(this,"Error","Please check the position data!");
            }
        }
    }
}


//      Add/Minus Joint Coord   //
void MainWindow::on_btnJointS__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(1,-1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);

}


void MainWindow::on_btnJointSp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(1,+1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointL__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(2,-1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointLp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(2,+1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointU__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(3,-1, var, socket);

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
    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointUp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(3,+1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointR__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(4,-1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointRp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(4,+1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointB__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(5,-1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointBp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(5,+1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointT__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(6,-1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnJointTp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);
    float speed = ui->txtJointSpeed->text().toDouble();
    float R1 = (ui->tbCurrentS->toPlainText().toDouble());
    float R2 = ui->tbCurrentL->toPlainText().toDouble();
    float R3 = ui->tbCurrentU->toPlainText().toDouble();
    float R4 = ui->tbCurrentR->toPlainText().toDouble();
    float R5 = ui->tbCurrentB->toPlainText().toDouble();
    float R6 = ui->tbCurrentT->toPlainText().toDouble();
    float var[7] = {speed, R1, R2, R3, R4, R5, R6};

    bool state = ControlLib::JointMove(6,+1, var, socket);

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

    ControlLib::Update_Pos(Mode,true,this->socket);
}

//******************************************//


//              Cart Tab                    //
//******************************************//
void MainWindow::on_btnForwardCart_clicked()
{
    if(ui->btnConnect->text() == "Connect")
        {
           QMessageBox::information(this,"Warning!!!","Pleased Connect UDP first then Turn on Servo");
        }
        else if(ui->btnServo->text() == "Servo ON")
        {
            QMessageBox::information(this,"Warning!!!","Pleased turn on Servo");
        }
        else
        {
            bool x,y,z,rx,ry,rz,v;
            x = ui->txtCartX->text().size()==0;
            y = ui->txtCartY->text().size()==0;
            z = ui->txtCartZ->text().size()==0;
            rx = ui->txtCartRoll->text().size()==0;
            ry = ui->txtCartPitch->text().size()==0;
            rz = ui->txtCartYaw->text().size()==0;
            v = ui->txtCartSpeed->text().size()==0;

            if(x||y||z||rx||ry||rz)
            {
                QMessageBox::critical(this,"Error","Please insert the position!");
            }
            else if (v)
            {
                QMessageBox::critical(this,"Error","Please insert the velocity!");
            }
            else
            {
                float X = ui->txtCartX->text().toDouble(&x);
                float Y = ui->txtCartY->text().toDouble(&y);
                float Z = ui->txtCartZ->text().toDouble(&z);
                float RX = ui->txtCartRoll->text().toDouble(&rx);
                float RY = ui->txtCartPitch->text().toDouble(&ry);
                float RZ = ui->txtCartYaw->text().toDouble(&rz);
                float speed = ui->txtCartSpeed->text().toDouble(&v);
                if(x && y && z && rx && ry && rz && v)
                {
                    ControlLib::Update_Pos(Mode,false,this->socket);

                    if(speed < 0 || speed > 100)
                    {
                        QMessageBox::warning(this,"Error","Please check the speed setpoint!");
                    }

                    const cv::Matx44d unit = {1,0,0,0,
                                              0,1,0,0,
                                              0,0,1,0,
                                              0,0,0,1};

//                    Convert::inverseKinematic(unit,)

                    bool state = this->socket->WritePosCart(speed,X,Y,Z,RX,RY,RZ);
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
                    ControlLib::Update_Pos(Mode,true,this->socket);
                }
                else
                {
                  QMessageBox::warning(this,"Error","Please check the position data!");
                }
            }
        }
}


//      Add/Minus Cartesian Coord   //

void MainWindow::on_btnPosX__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(1,-1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosXp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(1,+1, var, socket);//    if(this->socket->Get_PulsePos()->size() == 76)
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosY__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(2,-1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosYp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(2,+1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosZ__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(3,-1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosZp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(3,+1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosRx__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(4,-1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosRxp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(4,+1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosRy__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(5,-1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosRyp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(5,+1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosRz__clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(6,-1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}


void MainWindow::on_btnPosRzp_clicked()
{
    ControlLib::Update_Pos(Mode,false,this->socket);

    float speed = ui->txtCartSpeed->text().toDouble();
    float X = ui->tbCurrentPosX->toPlainText().toDouble();
    float Y = ui->tbCurrentPosY->toPlainText().toDouble();
    float Z = ui->tbCurrentPosZ->toPlainText().toDouble();
    float RX = ui->tbCurrentPosRoll->toPlainText().toDouble();
    float RY = ui->tbCurrentPosPitch->toPlainText().toDouble();
    float RZ = ui->tbCurrentPosYaw->toPlainText().toDouble();
    float var[7] = {speed, X, Y, Z, RX, RY, RZ};

    bool state = ControlLib::CartMove(6,+1, var, socket);
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

    ControlLib::Update_Pos(Mode,true,this->socket);
}
//******************************************//


void MainWindow::on_btnTeach_clicked()
{
    ControlLib::Update_Pos(Mode, false, this->socket);
    ControlLib::delay_ms(10);
    if(ui->btnConnect->text() == "Connect")
    {
       QMessageBox::information(this,"Warning!!!","Pleased Connect UDP first then Turn on Servo");
    }
    else if(ui->btnServo->text() == "Servo ON")
    {
        QMessageBox::information(this,"Warning!!!","Pleased turn on Servo");
    }
    else
    {
        if(ui->Man_tabTeaching->currentWidget() == ui->JointTab)
        {
            if(this->socket->Get_PulsePos()->size() == 76)
            {
                ControlLib::Teaching(ui->Man_tableJoint,ui->tbCurrentS, ui->tbCurrentL, ui->tbCurrentU,
                                     ui->tbCurrentR, ui->tbCurrentB, ui->tbCurrentT);
            }
            else
            {
                QMessageBox::warning(this,"Error","Cant get Joint Position");
            }


        }
        else
        {
            if(this->socket->Get_CartPos()->size() == 84)
            {
                ControlLib::Teaching(ui->Man_tableCart,ui->tbCurrentPosX, ui->tbCurrentPosY, ui->tbCurrentPosZ,
                                     ui->tbCurrentPosRoll, ui->tbCurrentPosPitch, ui->tbCurrentPosYaw);
            }
            else
            {
                QMessageBox::warning(this,"Error","Cant get Cart Position");
            }
        }
        ControlLib::Update_Pos(Mode, true, this->socket);
    }
}

void MainWindow::on_Man_tableJoint_cellClicked(int row, int column)
{
    bool ok;
    ok = ui->Man_tableJoint->item(row, 0)->text().size() == 0;
    if(!ok)
    {
        for(int i = 0; i < 6; i++)
        {
            teaching_pos[i] = ui->Man_tableJoint->item(row, i)->text().toFloat();
//            qDebug() << teaching_pos[i];
        }
    }
    else
    {
        ControlLib::AppOutput("No Value",this->ui->pteOutput);
    }
}


void MainWindow::on_Man_tableCart_cellClicked(int row, int column)
{
    bool ok;
    ok = ui->Man_tableCart->item(row, 0)->text().size() == 0;
    if(!ok)
    {
        for(int i = 0; i < 6; i++)
        {
            teaching_pos[i] = ui->Man_tableCart->item(row, i)->text().toFloat();
//            qDebug() << teaching_pos[i];
        }
    }
    else
    {
        ControlLib::AppOutput("No Value",this->ui->pteOutput);
    }
}

void MainWindow::on_btnDeleteRow_clicked()
{
//    ui->Man_tableJoint->removeRow(Selected_Row);
    if(ui->Man_tabTeaching->currentWidget() == ui->JointTab)
    {
        ui->Man_tableJoint->removeRow(ui->Man_tableJoint->currentRow());
    }

   if(ui->Man_tabTeaching->currentWidget() == ui->CartTab)
   {
       ui->Man_tableCart->removeRow(ui->Man_tableCart->currentRow());
   }
}

void MainWindow::on_btnForward_clicked()
{ ControlLib::delay_ms(200);

    ControlLib::Update_Pos(Mode,false,this->socket);
    if(ui->btnConnect->text() == "Connect")
    {
       QMessageBox::information(this,"Warning!!!","Pleased Connect UDP first then Turn on Servo");
    }
    else if(ui->btnServo->text() == "Servo ON")
    {
        QMessageBox::information(this,"Warning!!!","Pleased turn on Servo");
    }
    else
    {
        if(ui->Man_tabTeaching->currentWidget() == ui->CartTab)
        {
            bool state = this->socket->WritePosCart(15,teaching_pos[0],teaching_pos[1],teaching_pos[2],
                    teaching_pos[3],teaching_pos[4],teaching_pos[5]);
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
        else
        {
            bool state = this->socket->WritePosJoint(15,teaching_pos[0],teaching_pos[1],teaching_pos[2],
                    teaching_pos[3],teaching_pos[4],teaching_pos[5]);
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
    ControlLib::Update_Pos(Mode,true,this->socket);
}
//******************************************//


//              Serial Port                 //
//******************************************//
void MainWindow::on_btnSerialConnect_clicked()
{
    if(ui->btnSerialConnect->text() == "Connect")
    {
        stm = new serialPort(ui->cbSerialCom->currentText(), 115200);
        stm->SerialQPTE = this->ui->pteOutput;


        if(stm->Connect2STM(true))
        {
            stm->start();
            this->ui->btnSerialConnect->setText("Disconnect");
            this->ui->btnSerialConnect_2->setText("Serial Disconnect");
//            this->ui->btnSerialConnect_2->setEnabled(false);
            this->ui->lbStatus->setText("Connected");
            ControlLib::AppOutput("Serial Connected",this->ui->pteOutput);
            this->ui->btnADC_2->setEnabled(true);
            this->ui->btnGripper_2->setEnabled(true);
        }
        else
        {
            ControlLib::AppOutput("Can not connect to stm",this->ui->pteOutput);
        }
    }

    else if(ui->btnSerialConnect->text() == "Disconnect")
    {
        stm->startSend(2,-1000);
        stm->Connect2STM(false);
        stm->stop();
        this->ui->btnSerialConnect->setText("Connect");
        this->ui->btnSerialConnect_2->setText("Serial Connect");
//        this->ui->btnSerialConnect_2->setEnabled(true);
        this->ui->lbStatus->setText("Disconnected");
        ControlLib::AppOutput("Serial Disconnected",this->ui->pteOutput);
        delete stm;
        this->ui->btnADC_2->setEnabled(false);
        this->ui->btnGripper_2->setEnabled(false);
    }
}

void MainWindow::on_btnGripper_clicked()
{
    if(ui->btnSerialConnect->text() == "Disconnect")
    {
        float v = this->ui->txtGripperWidth->text().toFloat();
        if (v >= serialPort::GripperRelease && v <= serialPort::GripperMax)
        {
            stm->startSend(1,v);
        }
        else
        {
            QMessageBox::warning(this,"Error", "Out of work range");
        }
    }
    else
    {
        QMessageBox::warning(this,"Error", "Cant Connect to Port");
    }
}

void MainWindow::on_btnADC_clicked()
{
    if(ui->btnSerialConnect->text() == "Disconnect")
    {
        if(this->ui->btnADC->text() == "Level ON")
        {
            stm->startSend(2,1000);
            connect(stm,SIGNAL(readySensor(float)), this, SLOT(ADCDisplay(float)));
            this->ui->btnADC->setText("Level OFF");
        }
        else
        {
            this->ui->btnADC->setText("Level ON");
            stm->startSend(2, -1000);
            disconnect(stm,SIGNAL(readySensor(float)), this, SLOT(ADCDisplay(float)));
        }
    }
    else
    {
        QMessageBox::warning(this,"Error", "Cant Connect to Port");
    }
}

void MainWindow::ADCDisplay(float value)
{
    float tempValue = 0.0;
    tempValue = value;
//    if(tempValue < 0)
//    {
//        tempValue = 0.0;
//    }
    ControlLib::delay_ms(100);
    this->ui->txtADC->setText(QString::number(tempValue));
    this->ui->txtADC_2->setText(QString::number(tempValue));
}

//******************************************//
//******************************************//
//******************************************//


//              Auto Page                   //
//******************************************//
void MainWindow::on_btnStartProcess_clicked()
{
    if(this->ui->btnStartProcess->text()=="Start Process")
    {
        this->ui->btnStartProcess->setText("Continue");
        Waterprocess->start();
        Waterprocess->Startprocess();
    }

    if(this->ui->btnStartProcess->text()=="Continue")
    {
        if(bOoWater)
        {
            QMessageBox::StandardButton reply = QMessageBox::question(this, "Out of water", "Do you want to continue?",
                                                                      QMessageBox::Yes | QMessageBox::No);
            if(reply == QMessageBox::Yes)
            {
                Waterprocess->Startprocess();
            }
            else
            {
                this->ui->btnStartProcess->setEnabled(true);
                return;
            }
        }
        else
        {
            Waterprocess->Startprocess();
        }
    }

    ControlLib::AppOutput("Start process", this->ui->pteOutput);
    this->ui->btnStartProcess->setEnabled(false);
}

void MainWindow::on_btnCamera_clicked()
{
    if(this->ui->btnCamera->text() == "CAMERA ON")
    {
        this->camera->start();
        this->ui->btnCamera->setText("CAMERA OFF");
        ControlLib::AppOutput("Camera is ON", this->ui->pteOutput);

        this->ui->btnSavePC->setEnabled(true);
        this->ui->btnUpdateRefFrame->setEnabled(true);
        this->ui->btnStartProcess->setEnabled(true);
    }
    else if(this->ui->btnCamera->text() == "CAMERA OFF")
    {
        this->camera->stop();
        this->ui->btnCamera->setText("CAMERA ON");
//        this->ui->btnInit->setText("Initialize");
        ControlLib::AppOutput("Camera is OFF", this->ui->pteOutput);
//        UpdatePixmap();

        this->ui->btnSavePC->setEnabled(false);
        this->ui->btnUpdateRefFrame->setEnabled(false);
        this->ui->btnStartProcess->setEnabled(false);
    }
    else
    {
        QMessageBox::warning(this, "Warning", "Please initialize process frist");
    }
}


void MainWindow::on_btnRobotConnect_clicked()
{
    this->ui->btnRobotConnect->setEnabled(false);
    if(ui->btnRobotConnect->text()=="Robot Connect")
    {
        QHostAddress h;
        h.setAddress(this->ui->tbIPAddress_2->toPlainText());
        quint16 p = this->ui->tbPort_2->toPlainText().toUInt();
        socket = new udp(h,p);
        bool state = this->socket->ConnectMotoman();
        ControlLib::delay_ms(200);

        Mode = 0;
        // Lay Signal tu Thread de doc vi tri cua Robot
        connect(this->timerTimeout, SIGNAL(timeout()), this->socket, SLOT(OFFTrigger()));
        ControlLib::Update_Pos(Mode,false, socket);
        ControlLib::delay_ms(10);

        bool state1 = this->socket->OnServo();
        ControlLib::delay_ms(1500);
        this->timerTimeout->start(1000);
        ControlLib::delay_ms(200);

//        int32_t status;
//        bool readstatus = this->socket->ReadStatus(&status);
//        ControlLib::delay_ms(500);

        if(state1)
        {
            this->timerTimeout->stop();
            this->socket->start();
            Mode = 0;
            connect(this->socket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_Data()));
            ControlLib::Update_Pos(Mode,true,this->socket);
            this->Home();
            ControlLib::delay_ms(200);
            this->ui->btnRobotConnect->setText("Robot Disconnect");
            this->ui->btnConnect->setText("Disconnect");
            this->ui->btnConnect->setEnabled(false);
            ControlLib::AppOutput("Robot Connected",this->ui->pteOutput);
            this->ui->btnHome->setEnabled(true);
            this->ui->btnServo->setText("Servo OFF");
        }
        else
        {
            ControlLib::Update_Pos(Mode,false,this->socket);
            ControlLib::delay_ms(30);
            QMessageBox::warning(this, "Warning", "Cant Connect");
//            disconnect(this->socket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_Data()));
            this->socket->Stopthread();
            ControlLib::delay_ms(30);
            this->socket->DisconnectMotoman();
            this->ui->btnRobotConnect->setText("Robot Connect");
            this->ui->btnConnect->setText("Connect");
            this->ui->btnConnect->setEnabled(true);
            disconnect(this->timerTimeout, SIGNAL(timeout()), this->socket, SLOT(OFFTrigger()));
//            delete this->socket;
            ControlLib::AppOutput("Can not connect to Robot",this->ui->pteOutput);
        }
    }

    else
    {
        this->Home();
        ControlLib::delay_ms(500);
        ControlLib::Update_Pos(Mode, false, socket);
        ControlLib::delay_ms(10);
        this->socket->Stopthread();
        disconnect(this->socket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_Data()));
        bool state1 = this->socket->OffServo();
        ControlLib::delay_ms(1500);
//        int32_t status1;
//        bool readstatus = this->socket->ReadStatus(&status1);
//        ControlLib::delay_ms(300);
        this->timerTimeout->start(1000);
        if(state1)
        {
            this->timerTimeout->stop();
            this->ui->btnRobotConnect->setText("Robot Connect");
            this->ui->btnConnect->setText("Connect");
            this->ui->btnConnect->setEnabled(true);
            disconnect(this->timerTimeout, SIGNAL(timeout()), this->socket, SLOT(OFFTrigger()));
            ControlLib::AppOutput("Robot Disconnected",this->ui->pteOutput);
            this->ui->btnHome->setEnabled(false);
            bool state = this->socket->DisconnectMotoman();
            ControlLib::delay_ms(200);
            this->ui->btnServo->setText("Servo ON");
        }
        else
        {
            connect(this->socket,SIGNAL(Sig_ReadRobotPos()),this,SLOT(Update_Data()));
            QMessageBox::warning(this,"Warning!!!","Cant connect");
        }
    }
    this->ui->btnRobotConnect->setEnabled(true);
}

void MainWindow::on_btnHome_clicked()
{
    if(this->ui->btnRobotConnect->text() == "Robot Disconnect")
    {
        this->Home();
        ControlLib::delay_ms(500);
    }
    else
    {
        QMessageBox::warning(this, "Warning", "Please connect to Robot");
    }
}

void MainWindow::on_btnSerialConnect_2_clicked()
{
    this->ui->btnSerialConnect_2->setEnabled(false);
    if(this->ui->btnSerialConnect_2->text() == "Serial Connect")
    {
        int baud = this->ui->cbSerialBaudrate->currentText().toInt();
        stm = new serialPort(ui->cbSerialCom_2->currentText(),baud);

        if(stm->Connect2STM(true))
        {
            stm->start();
            this->ui->btnSerialConnect_2->setText("Serial Disconnect");
            this->ui->btnSerialConnect->setText("Disconnect");
//            this->ui->btnSerialConnect->setEnabled(false);
            this->ui->lbStatus->setText("Connected");
            ControlLib::AppOutput("Serial Connected",this->ui->pteOutput);
            this->ui->btnADC_2->setEnabled(true);
            this->ui->btnGripper_2->setEnabled(true);
            connect(stm,SIGNAL(readySensor(float)), this, SLOT(ADCDisplay(float)));
        }
        else
        {
            ControlLib::AppOutput("Cannot connect to Port",this->ui->pteOutput);
            delete stm;
        }
    }

    else if(ui->btnSerialConnect_2->text() == "Serial Disconnect")
    {
        disconnect(stm,SIGNAL(readySensor(float)), this, SLOT(ADCDisplay(float)));
        stm->startSend(2,-1000);
        stm->Connect2STM(false);
        stm->stop();
        this->ui->btnSerialConnect_2->setText("Serial Connect");
        this->ui->btnSerialConnect->setText("Connect");
//        this->ui->btnSerialConnect->setEnabled(true);
        ControlLib::AppOutput("Serial Disconnected",this->ui->pteOutput);
        this->ui->lbStatus->setText("Disconnected");
        delete stm;

        this->ui->btnADC_2->setEnabled(false);
        this->ui->btnGripper_2->setEnabled(false);
    }
    this->ui->btnSerialConnect_2->setEnabled(true);
}

void MainWindow::on_btnGripper_2_clicked()
{
    if(ui->btnSerialConnect_2->text() == "Serial Disconnect")
    {
        float v = this->ui->txtGripperWidth_2->text().toFloat();
        if (v >= serialPort::GripperRelease && v <= serialPort::GripperMax)
        {
            stm->startSend(1,v);
        }
        else
        {
            QMessageBox::warning(this,"Error", "Out of work range");
        }
    }
    else
    {
        QMessageBox::warning(this,"Error", "Cant Connect to Port");
    }
}




void MainWindow::on_btnADC_2_clicked()
{
    if(ui->btnSerialConnect_2->text() == "Serial Disconnect")
    {
        if(this->ui->btnADC_2->text() == "Level ON")
        {
            stm->startSend(2,1000);
            connect(stm,SIGNAL(readySensor(float)), this, SLOT(ADCDisplay(float)));
            this->ui->btnADC_2->setText("Level OFF");
        }
        else
        {
            this->ui->btnADC_2->setText("Level ON");
            stm->startSend(2, -1000);
//            disconnect(stm,SIGNAL(readySensor(float)), this, SLOT(ADCDisplay(float)));
        }
    }
    else
    {
        QMessageBox::warning(this,"Error", "Cant Connect to Port");
    }
}

void MainWindow::on_btnInit_clicked()
{
    if(this->ui->btnRobotConnect->text() == "Robot Disconnect"
       && this->ui->btnSerialConnect_2->text() == "Serial Disconnect")
    {
        if(this->ui->btnInit->text() == "Initialize")
        {
            this->ui->btnInit->setText("Re-Initialize");
            this->camera = new RealsenseCamera(640, 480);
        }

        ControlLib::AppOutput("Initialize",this->ui->pteOutput);

        QPalette pal = this->ui->led_PE->palette();
        pal.setColor(QPalette::Button, QColor(Qt::gray));
        this->ui->led_PE->setAutoFillBackground(true);
        this->ui->led_PE->setPalette(pal);
        this->ui->led_PE->update();

        QPalette pal1 = this->ui->led_OOW->palette();
        pal1.setColor(QPalette::Button, QColor(Qt::gray));
        this->ui->led_OOW->setAutoFillBackground(true);
        this->ui->led_OOW->setPalette(pal);
        this->ui->led_OOW->update();

        this->readfile_mainwindow();
        this->camera->loadCameraConfigPath(this->cameraFile);
        this->camera->initialize_bgFilter(this->colorCalibFile);
        this->camera->initialize_pcCreator(0.17, 1.0);
        bool USE_CUDA = (bool)cv::cuda::getCudaEnabledDeviceCount();
        std::cout << USE_CUDA << std::endl;
        this->camera->initialize_objDetector(this->yoloModelFile, this->yoloClassFile,
                                             3, USE_CUDA, 0.6, 0.5, 0.5, 0.4);
        connect(this->camera, SIGNAL(detectReady(QImage)), this, SLOT(show_lbVideoCapture(QImage)));
        connect(this->camera, SIGNAL(maskReady(QImage)), this, SLOT(show_lbVideoCapture_2(QImage)));
        Waterprocess = new process(Mode, this->socket, this->stm, this->camera,
                                   this->ui->Config_tbPosefile->toPlainText().toStdString(),
                                   this->ui->Config_tbBottlefile->toPlainText().toStdString(),
                                   this->ui->Config_tbCupfile->toPlainText().toStdString());
        Waterprocess->ProcessTxt = this->ui->pteOutput;
        connect(this->Waterprocess, SIGNAL(Sig_finishProcess(bool)), this->ui->btnStartProcess, SLOT(setEnabled(bool)));
        connect(this->Waterprocess, SIGNAL(Sig_finishProcess(bool)), this, SLOT(SLOT_finishProcess(bool)));
        connect(this->Waterprocess, SIGNAL(finish(int32_t*)), this, SLOT(showPose(int32_t*)));
        connect(this->Waterprocess, SIGNAL(Sig_poseERROR()), this, SLOT(Sig_poseERROR()));
        connect(this->Waterprocess, SIGNAL(Sig_OOW()), this, SLOT(Sig_OUTOFWATER()));

        this->ui->btnCamera->setEnabled(true);
    }
    else
    {
       QMessageBox::warning(this, "Error", "Please turn on Robot and Serial Port");
    }
}

void MainWindow::SLOT_finishProcess(bool status)
{
    ControlLib::AppOutput("Process finished",this->ui->pteOutput);
}
void MainWindow::Sig_poseERROR()
{
    QPalette pal = this->ui->led_PE->palette();
    pal.setColor(QPalette::Button, QColor(Qt::red));
    this->ui->led_PE->setAutoFillBackground(true);
    this->ui->led_PE->setPalette(pal);
    this->ui->led_PE->update();
}

void MainWindow::Sig_OUTOFWATER()
{
    QPalette pal = this->ui->led_OOW->palette();
    pal.setColor(QPalette::Button, QColor(Qt::yellow));
    this->ui->led_OOW->setAutoFillBackground(true);
    this->ui->led_OOW->setPalette(pal);
    this->ui->led_OOW->update();
    ControlLib::AppOutput("Out of water",this->ui->pteOutput);
    bOoWater = true;
}

void MainWindow::on_led_PE_clicked()
{
    QPalette pal = this->ui->led_PE->palette();
    pal.setColor(QPalette::Button, QColor(Qt::gray));
    this->ui->led_PE->setAutoFillBackground(true);
    this->ui->led_PE->setPalette(pal);
    this->ui->led_PE->update();
}

void MainWindow::on_led_OOW_clicked()
{
    QPalette pal = this->ui->led_OOW->palette();
    pal.setColor(QPalette::Button, QColor(Qt::gray));
    this->ui->led_OOW->setAutoFillBackground(true);
    this->ui->led_OOW->setPalette(pal);
    this->ui->led_OOW->update();
    bOoWater = false;
}

void MainWindow::showPose(int32_t *poses)
{
    ControlLib::DisplayJoint(this->ui->tbReadyPickPoseX, this->ui->tbReadyPickPoseY, this->ui->tbReadyPickPoseZ,
                             this->ui->tbReadyPickPoseRoll, this->ui->tbReadyPickPosePitch, this->ui->tbReadyPickPoseYaw,
                             poses);
    ControlLib::DisplayJoint(this->ui->tbPickPoseX, this->ui->tbPickPoseY, this->ui->tbPickPoseZ,
                             this->ui->tbPickPoseRoll, this->ui->tbPickPosePitch, this->ui->tbPickPoseYaw,
                             &poses[6]);
}

void MainWindow::show_lbVideoCapture(QImage img)
{
    ControlLib::ShowImage(this->ui->lbVideoCapture, img, 640, 480);
}

void MainWindow::show_lbVideoCapture_2(QImage img)
{
    ControlLib::ShowImage(this->ui->lbVideoCapture_2, img, 640, 480);
}

void MainWindow::on_btnEmer_clicked()
{
    this->ui->btnEmer->setEnabled(false);
    if(this->ui->btnEmer->text() == "EMERGENCY ON")
    {
        this->socket->OffServo();
        this->ui->btnEmer->setText("EMERGENCY OFF");
        Waterprocess->stopThread();
        ControlLib::delay_ms(200);
//        delete Waterprocess;
    }
    else
    {
        this->ui->btnEmer->setText("EMERGENCY ON");
        this->socket->OnServo();
        ControlLib::delay_ms(1000);
        this->Home();
        ControlLib::delay_ms(200);
        Waterprocess->JOB_Clear(2);
        ControlLib::delay_ms(100);
        Waterprocess->start();
    }
    this->ui->btnStartProcess->setEnabled(true);
    this->ui->btnEmer->setEnabled(true);
}


//              Config Page                 //
//******************************************//
void MainWindow::on_Config_btnHome_clicked()
{
    if(this->ui->btnRobotConnect->text() == "Robot Disconnect")
    {
        ControlLib::Update_Pos(Mode,false,this->socket);
        bool state = this->socket->WritePosJoint(10,0,0,0,0,0,0);
        ControlLib::delay_ms(500);
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
        ControlLib::Update_Pos(Mode,true,this->socket);
    }
    else
    {
        QMessageBox::warning(this, "Warning", "Please connect to Robot");
    }
}


void MainWindow::on_Config_btnPoseForm_clicked()
{
    if(this->ui->btnInit->text() == "Re-Initialize")
    {
        this->camera->stop();
        delete this->camera;
        this->ui->btnInit->setText("Initialize");
    }

    this->hide();
    CalibposeForm->IPAddress = this->ui->tbIPAddress_2->toPlainText();
    CalibposeForm->IPPort = this->ui->tbPort_2->toPlainText().toUInt();
    CalibposeForm->QPlText = this->ui->pteOutput;
    CalibposeForm->show();
}


void MainWindow::on_Config_btnColorForm_clicked()
{
    if(this->ui->btnInit->text() == "Re-Initialize")
    {
        this->camera->stop();
        delete this->camera;
        this->ui->btnInit->setText("Initialize");
    }
    this->hide();
    CalibColorForm->show();
}

void MainWindow::on_Config_btnCameraBrowser_clicked()
{
    this->ui->Config_tbCamerafile->setText(QFileDialog::getOpenFileName(this, "Choose Camera File", QDir::homePath()));
}

void MainWindow::on_Config_btnPoseBrowser_clicked()
{
    this->ui->Config_tbPosefile->setText(QFileDialog::getExistingDirectory(this, "Choose Pose Calib File", QDir::homePath()));
}


void MainWindow::on_Config_btnColorBrowser_clicked()
{
    this->ui->Config_tbColorfile->setText(QFileDialog::getOpenFileName(this, "Choose Color Calib File", QDir::homePath()));
}


void MainWindow::on_Config_btnModelBrowser_clicked()
{
    this->ui->Config_tbModelfile->setText(QFileDialog::getOpenFileName(this, "Choose YOLO Model File", QDir::homePath()));
}


void MainWindow::on_Config_btnClassBrowser_clicked()
{
    this->ui->Config_tbClassfile->setText(QFileDialog::getOpenFileName(this, "Choose YOLO Class File", QDir::homePath()));
}

void MainWindow::on_Config_btnBottleBrowser_clicked()
{
    this->ui->Config_tbBottlefile->setText(QFileDialog::getOpenFileName(this, "Choose 3D Bottle Model File", QDir::homePath()));
}


void MainWindow::on_Config_btnCupBrowser_clicked()
{
    this->ui->Config_tbCupfile->setText(QFileDialog::getOpenFileName(this, "Choose 3D Cup Model File", QDir::homePath()));
}


void MainWindow::on_btnUpdateRefFrame_clicked()
{
    if(this->camera->isRunning)
    {
        this->camera->updateRefFrame();
        ControlLib::AppOutput("Update Ref Frame",this->ui->pteOutput);
    }
    else
    {
        std::cout << "Please open Camera" << std::endl;
    }
}


void MainWindow::on_btnSavePC_clicked()
{
    this->camera->savePC();
    ControlLib::AppOutput("Save Point Cloud",this->ui->pteOutput);
}


void MainWindow::on_btnBrowserJoint_clicked()
{
    this->ui->tbSaveJointURL->setText(QFileDialog::getExistingDirectory(this, "Choose directory", QDir::homePath()));
}


void MainWindow::on_btnBrowserCart_clicked()
{
    this->ui->tbSaveCartURL->setText(QFileDialog::getExistingDirectory(this, "Choose directory", QDir::homePath()));
}


void MainWindow::on_btnBrowser_clicked()
{
    this->ui->tbSaveURL->setText(QFileDialog::getExistingDirectory(this, "Choose directory", QDir::homePath()));
}

void MainWindow::on_pushButton_7_clicked()
{
    stm->startSend(1,serialPort::GripperRelease);
    ControlLib::delay_ms(1000);
}

void MainWindow::on_pushButton_clicked()
{
    this->camera->startProcess();
}


void MainWindow::on_btnInit_Eva_clicked()
{
    cv::Matx44d tag2obj = {1, 0, 0, 0,
                          0, 1, 0, 0.105,
                          0, 0, 1, 0.035,
                          0, 0, 0, 1};
    this->eva = new EvaluatePose("/home/phuongdoan/Code/ThesisGUI/config.json",
                                 "/home/phuongdoan/Code/ThesisGUI/GroundTruthPose",
                                 tag2obj);

    this->eva->start();
}


void MainWindow::on_btnGetPose_clicked()
{
    this->eva->startTrigger();
}


