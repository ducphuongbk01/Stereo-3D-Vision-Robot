#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QThread>
#include <QSerialPort>
#include <QPlainTextEdit>
#include <QMetaType>
#include <iostream>
#include <fstream>
#include "controllib.h"

struct TxSTM
{
    uint8_t STX = 0x02;
    char cmd[4];
    uint8_t data[4];
    uint8_t check_sum;
    uint8_t ETX = 0x03;
};

Q_DECLARE_METATYPE(TxSTM);

class serialPort : public QThread
{
    Q_OBJECT
private:
    QSerialPort *serial;
    bool servoSend = false;
    bool adcSend = false;
    bool isRunning = false;
    bool flag_connect = false;

    float data;

    void Float2byte(uint8_t bArray[4], float value);
    void Bytes2float(uint8_t *bArray, float *value);

    void ProtocolFrame(uint8_t *y, int stmMode, float v);
    void Send2STM(uint8_t *buffer);
    uint8_t *Qbyte2uint8t(QByteArray &buffer);
    bool CheckOpen();
    uint8_t checksum(uint8_t *addr, uint8_t count);
    bool check_checksum(uint8_t *addr, uint8_t count);

public:
    serialPort();
    serialPort(QString Com, int Baud);
    ~serialPort();

    void run();
    void stop();

    void startSend(int sendMode, float fdata);
    bool Connect2STM(bool isConnect);
    void ConfigSerial(QString Com);

    static const int GripperNutri;
    static const int GripperTH;
    static const int GripperWhiteCup;
    static const int GripperPinkCup;
    static const int GripperRelease;
    static const int GripperMax;

    QPlainTextEdit *SerialQPTE;

signals:
    void readySensor(float value);

    void readyServo();

    void readyADC();

public slots:
    void CheckFrame();

    void SendServoWidth();

    void SendADCrequest();
};

#endif // SERIALPORT_H
