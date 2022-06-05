#ifndef CONTROLLIB_H
#define CONTROLLIB_H

#include "udp.h"
#include "convert.h"
#include <QTextBrowser>
#include <QTableWidget>
#include <QTimer>
#include <QSerialPort>
#include <QImage>
#include <QLabel>
#include <QPlainTextEdit>


namespace ControlLib
{
void delay_ms(int n);

void DisplayPosition(QTextBrowser *x, QTextBrowser *y, QTextBrowser *z,
                     QTextBrowser *r, QTextBrowser *p, QTextBrowser *ya, int32_t *point);

void DisplayJoint(QTextBrowser *R1, QTextBrowser *R2, QTextBrowser *R3,
                     QTextBrowser *R4, QTextBrowser *R5, QTextBrowser *R6, int32_t *point);

bool JointMove(int Joint_stt, int Mode, float *var, udp *CtrlSocket);

bool CartMove(int Cart_stt, int Mode, float *var, udp *CtrlSocket);

bool Check_Workspace(int Mode, int32_t Pos1, int32_t Pos2, int32_t Pos3, int32_t Pos4, int32_t Pos5, int32_t Pos6);

void Teaching(QTableWidget *Table, QTextBrowser *R1, QTextBrowser *R2, QTextBrowser *R3,
              QTextBrowser *R4, QTextBrowser *R5, QTextBrowser *R6);

void Update_Pos(int mode, bool sig, udp *CtrlSocket);

bool GoHome(int Mode, udp *CtrlSocket);

void ShowImage(QLabel *imgLabel, QImage img, int Width, int Height);

void AppOutput(QString text, QPlainTextEdit *plaintext);

};

#endif // CONTROLLIB_H
