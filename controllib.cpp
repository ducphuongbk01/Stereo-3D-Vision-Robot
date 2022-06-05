#include "controllib.h"
#include "convert.h"
#include "udp.h"
#include <iostream>
#include <QTime>
#include <QSerialPort>
#include <QPixmap>


void ControlLib::delay_ms(int n)
{
    QTime dieTime= QTime::currentTime().addMSecs(n);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
};

void ControlLib::DisplayPosition(QTextBrowser *x, QTextBrowser *y, QTextBrowser *z,
                     QTextBrowser *r, QTextBrowser *p, QTextBrowser *ya, int32_t *point)
{
    x->setText(QString::number((double)point[0]/1000.0));
    y->setText(QString::number((double)point[1]/1000.0));
    z->setText(QString::number((double)point[2]/1000.0));
    r->setText(QString::number((double)point[3]/10000.0));
    p->setText(QString::number((double)point[4]/10000.0));
    ya->setText(QString::number((double)point[5]/10000.0));

    x->setAlignment(Qt::AlignCenter);
    y->setAlignment(Qt::AlignCenter);
    z->setAlignment(Qt::AlignCenter);
    r->setAlignment(Qt::AlignCenter);
    p->setAlignment(Qt::AlignCenter);
    ya->setAlignment(Qt::AlignCenter);
}

void ControlLib::DisplayJoint(QTextBrowser *R1, QTextBrowser *R2, QTextBrowser *R3,
                     QTextBrowser *R4, QTextBrowser *R5, QTextBrowser *R6, int32_t *point)
{
    R1->setText(QString::number((double)point[0]/(udp::PULSE_PER_DEGREE_S)));
    R2->setText(QString::number((double)point[1]/(udp::PULSE_PER_DEGREE_L)));
    R3->setText(QString::number((double)point[2]/(udp::PULSE_PER_DEGREE_U)));
    R4->setText(QString::number((double)point[3]/(udp::PULSE_PER_DEGREE_RBT)));
    R5->setText(QString::number((double)point[4]/(udp::PULSE_PER_DEGREE_RBT)));
    R6->setText(QString::number((double)point[5]/(udp::PULSE_PER_DEGREE_RBT)));

    R1->setAlignment(Qt::AlignCenter);
    R2->setAlignment(Qt::AlignCenter);
    R3->setAlignment(Qt::AlignCenter);
    R4->setAlignment(Qt::AlignCenter);
    R5->setAlignment(Qt::AlignCenter);
    R6->setAlignment(Qt::AlignCenter);
}


bool ControlLib::JointMove(int Joint_stt, int Mode, float *var, udp *CtrlSocket)
{
    float J_Speed = var[0];
    float J_R1, J_R2, J_R3, J_R4, J_R5, J_R6;
    switch(Joint_stt)
    {
        case 1:
            J_R1 =    var[1] + J_Speed*Mode;
            J_R2 =    var[2];
            J_R3 =    var[3];
            J_R4 =    var[4];
            J_R5 =    var[5];
            J_R6 =    var[6];
            break;
        case 2:
            J_R1 =    var[1];
            J_R2 =    var[2] + J_Speed*Mode;
            J_R3 =    var[3];
            J_R4 =    var[4];
            J_R5 =    var[5];
            J_R6 =    var[6];
            break;
        case 3:
            J_R1 =    var[1];
            J_R2 =    var[2];
            J_R3 =    var[3] + J_Speed*Mode;
            J_R4 =    var[4];
            J_R5 =    var[5];
            J_R6 =    var[6];
            break;
        case 4:
            J_R1 =    var[1];
            J_R2 =    var[2];
            J_R3 =    var[3];
            J_R4 =    var[4] + J_Speed*Mode;
            J_R5 =    var[5];
            J_R6 =    var[6];
            break;
        case 5:
            J_R1 =    var[1];
            J_R2 =    var[2];
            J_R3 =    var[3];
            J_R4 =    var[4];
            J_R5 =    var[5] + J_Speed*Mode;
            J_R6 =    var[6];
            break;
        case 6:
            J_R1 =    var[1];
            J_R2 =    var[2];
            J_R3 =    var[3];
            J_R4 =    var[4];
            J_R5 =    var[5];
            J_R6 =    var[6] + J_Speed*Mode;
            break;
        default:
            J_R1 =    var[1];
            J_R2 =    var[2];
            J_R3 =    var[3];
            J_R4 =    var[4];
            J_R5 =    var[5];
            J_R6 =    var[6];
            break;
    }

    var[1] = J_R1;
    var[2] = J_R2;
    var[3] = J_R3;
    var[4] = J_R4;
    var[5] = J_R5;
    var[6] = J_R6;

    std::vector<float> joints_limit = {J_R1,J_R2,J_R3,J_R4,J_R5,J_R6};
    bool state = false;

    bool WorkSpace = Convert::JointWorkspace(joints_limit);
    if(WorkSpace)
    {
        state = CtrlSocket->WritePosJoint(J_Speed, J_R1, J_R2, J_R3, J_R4, J_R5, J_R6);
        ControlLib::delay_ms(200);
        return state;
    }
    else
    {
        return false;
    }

}


bool ControlLib::CartMove(int Cart_stt, int Mode, float *var, udp *CtrlSocket)
{
    float C_Speed = var[0];
    float C_X, C_Y, C_Z, C_Roll, C_Pitch, C_Yaw;
    switch(Cart_stt)
    {
        case 1:
            C_X =       var[1] + C_Speed*Mode;
            C_Y =       var[2];
            C_Z =       var[3];
            C_Roll =    var[4];
            C_Pitch =   var[5];
            C_Yaw =     var[6];
            break;
        case 2:
            C_X =       var[1];
            C_Y =       var[2]+ C_Speed*Mode;
            C_Z =       var[3];
            C_Roll =    var[4];
            C_Pitch =   var[5];
            C_Yaw =     var[6];
            break;
        case 3:
            C_X =       var[1];
            C_Y =       var[2];
            C_Z =       var[3] + C_Speed*Mode;
            C_Roll =    var[4];
            C_Pitch =   var[5];
            C_Yaw =     var[6];
            break;
        case 4:
            C_X =       var[1];
            C_Y =       var[2];
            C_Z =       var[3];
            C_Roll =    var[4] + C_Speed*Mode;
            C_Pitch =   var[5];
            C_Yaw =     var[6];
            break;
        case 5:
            C_X =       var[1];
            C_Y =       var[2];
            C_Z =       var[3];
            C_Roll =    var[4];
            C_Pitch =   var[5] + C_Speed*Mode;
            C_Yaw =     var[6];
            break;
        case 6:
            C_X =       var[1];
            C_Y =       var[2];
            C_Z =       var[3];
            C_Roll =    var[4];
            C_Pitch =   var[5];
            C_Yaw =     var[6] + C_Speed*Mode;
            break;
        default:
            C_X =       var[1];
            C_Y =       var[2];
            C_Z =       var[3];
            C_Roll =    var[4];
            C_Pitch =   var[5];
            C_Yaw =     var[6];
            break;
    }

    bool state = CtrlSocket->WritePosCart(C_Speed, C_X, C_Y, C_Z, C_Roll, C_Pitch, C_Yaw);
    return state;
}


bool ControlLib::Check_Workspace(int Mode, int32_t Pos1, int32_t Pos2, int32_t Pos3, int32_t Pos4, int32_t Pos5, int32_t Pos6)
{
    switch(Mode)
    {
        case 1:
            if(Pos1 > 170 || Pos1 < -170)
            {
                return false;
            }
            if(Pos2 > 90 || Pos2 < -85)
            {
                return false;
            }
            if(Pos3 > 90 || Pos3 < -50)
            {
                return false;
            }
            if(Pos4 > 140 || Pos4 < -140)
            {
                return false;
            }
            if(Pos5 > 210 || Pos5 < -30)
            {
                return false;
            }
            if(Pos6 > 360 || Pos6 < -360)
            {
                return false;
            }
            break;

        case 2:
        return true;

        break;
    }
    return true;
}


void ControlLib::Teaching(QTableWidget *Table, QTextBrowser *R1, QTextBrowser *R2, QTextBrowser *R3,
              QTextBrowser *R4, QTextBrowser *R5, QTextBrowser *R6)
{
    int Num = Table->rowCount();
    Table->setRowCount(Num+1);
    Table->setColumnCount(6);
    Table->setItem(Num,0,new QTableWidgetItem(QString::number(R1->toPlainText().toDouble())));
    Table->setItem(Num,1,new QTableWidgetItem(QString::number(R2->toPlainText().toDouble())));
    Table->setItem(Num,2,new QTableWidgetItem(QString::number(R3->toPlainText().toDouble())));
    Table->setItem(Num,3,new QTableWidgetItem(QString::number(R4->toPlainText().toDouble())));
    Table->setItem(Num,4,new QTableWidgetItem(QString::number(R5->toPlainText().toDouble())));
    Table->setItem(Num,5,new QTableWidgetItem(QString::number(R6->toPlainText().toDouble())));

//    Table->model()->setData()
}


void ControlLib:: Update_Pos(int mode, bool sig, udp *CtrlSocket)
{
    if(CtrlSocket == NULL)
    {
        qDebug() << "Please create a socket";
    }
    else
    {
        switch (mode)
        {
            case 0:
            {
                CtrlSocket->readPos = sig;
                CtrlSocket->readJoint = false;
                CtrlSocket->readCart = false;
                break;
            }

            case 1:
            {
                CtrlSocket->readCart = sig;
                CtrlSocket->readJoint = false;
                CtrlSocket->readPos = false;
                break;
            }

            case 2:
            {
                CtrlSocket->readJoint = sig;
                CtrlSocket->readCart = false;
                CtrlSocket->readPos = false;
                break;
            }

            case 3:
            {
                CtrlSocket->readByte = sig;
                CtrlSocket->readCart = false;
                CtrlSocket->readPos = false;
                CtrlSocket->readJoint = false;
                break;
            }

            default:
                break;
        }
    }
}


bool ControlLib::GoHome(int Mode, udp *CtrlSocket)
{
    ControlLib::Update_Pos(Mode,false,CtrlSocket);
    bool state = CtrlSocket->WritePosJoint(10,udp::Home_1[0],udp::Home_1[1],udp::Home_1[2],udp::Home_1[3],udp::Home_1[4],udp::Home_1[5]);
    ControlLib::delay_ms(500);
    ControlLib::Update_Pos(Mode,true,CtrlSocket);
    return state;
}


void ControlLib::ShowImage(QLabel *imgLabel, QImage img, int Width, int Height)
{
    imgLabel->setPixmap(QPixmap::fromImage(img).scaled(Width,Height));
}

void ControlLib::AppOutput(QString text, QPlainTextEdit *plaintext)
{
    plaintext->moveCursor(QTextCursor::End);
    QTime qtcurrenttime = QTime::currentTime();
    QString time = qtcurrenttime.toString();
    QString TextOut = time + ": " + text;
    plaintext->insertPlainText(TextOut);
    plaintext->insertPlainText("\n");
}
