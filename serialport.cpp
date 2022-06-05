#include "serialport.h"

float pre_value;

serialPort::serialPort()
{
    serial = new QSerialPort;
    SerialQPTE = new QPlainTextEdit;
}

serialPort::serialPort(QString Com, int Baud)
{
    serial = new QSerialPort(this);
    serial->setPortName(Com);
    serial->setBaudRate(Baud);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    SerialQPTE = new QPlainTextEdit;
}

serialPort::~serialPort()
{

}


bool serialPort::Connect2STM(bool isConnect)
{
    if(isConnect)
    {
        if(this->serial->open(QSerialPort::ReadWrite))
        {
            if(this->CheckOpen())
            {
                this->isRunning = true;
                connect(serial, SIGNAL(readyRead()), this, SLOT(CheckFrame()));
                connect(this, SIGNAL(readyServo()), this, SLOT(SendServoWidth()));
                connect(this, SIGNAL(readyADC()), this, SLOT(SendADCrequest()));
                return true;
            }
            else return false;
        }
        else return false;
    }
    else
    {
        this->serial->close();
        return true;
    }
}

bool serialPort::CheckOpen()
{
    uint8_t sendbyte[11];
    this->ProtocolFrame(sendbyte, 0, 0);
    ControlLib::delay_ms(100);
    this->CheckFrame();
    return flag_connect;
}

void serialPort::ConfigSerial(QString Com)
{
    serial->setPortName(Com);
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
}

void serialPort::Bytes2float(uint8_t *bArray, float *value)
{
    *value = *(float *)bArray;
}

void serialPort::Float2byte(uint8_t bArray[4], float value)
{
    union
    {
        float a;
        unsigned char byte[4];
    } convert;
    convert.a = value;
    memcpy(bArray, convert.byte, 4);
}

void serialPort::Send2STM(uint8_t *buffer)
{
    QByteArray send;
    send.resize(11);


    for(int i = 0; i < 11; i++)
    {
        send[i] = buffer[i];
    }

    serial->write(send);

    ControlLib::AppOutput("Frame was sent", SerialQPTE);
}

uint8_t *serialPort::Qbyte2uint8t(QByteArray &buffer)
{
    uint8_t *send;

    for(int i = 0; i < 11; i++)
    {
        send[i] = buffer[i];
    }
    return send;

}

void serialPort::ProtocolFrame(uint8_t *y, int stmMode, float v)
{
    TxSTM stmframe;

    switch(stmMode)
    {
        case(0): // Check Connect
        {
            char cmd[4] = {'G', 'C', 'O', 'N'};
            memcpy(stmframe.cmd, &cmd, 4);
            memcpy(y,&stmframe,11);
            stmframe.check_sum = (uint8_t)this->checksum(&y[1], 8);
            memcpy(y,&stmframe,11);
            break;
        }

        case(1): // Send Gripper Width
        {
            char cmd[4] = {'G', 'W', 'M', 'G'};
            memcpy(stmframe.cmd, &cmd, 4);
            Float2byte(stmframe.data, v);
            memcpy(y,&stmframe,11);
            stmframe.check_sum = (uint8_t)this->checksum(&y[1], 8);
            memcpy(y,&stmframe,11);
            break;
        }

        case(2):    // Ask ADC
        {
            char cmd[4] = {'G', 'A', 'D', 'C'};
            memcpy(stmframe.cmd, &cmd, 4);
            Float2byte(stmframe.data, v);
            memcpy(y,&stmframe,11);
            stmframe.check_sum = (uint8_t)this->checksum(&y[1], 8);
            memcpy(y,&stmframe,11);
            break;
        }
    }

    Send2STM(y);
}

void serialPort::run()
{
    if(!this->isRunning)
    {
        std::cout << "Cannot open Serial." << std::endl;
        ControlLib::AppOutput( "Cannot open Serial.",SerialQPTE);
        return;
    }

    while(this->isRunning)
    {
        if(this->servoSend)
        {
            servoSend = false;
            Q_EMIT readyServo();
        }

        if(this->adcSend)
        {
            adcSend = false;
            Q_EMIT readyADC();
        }
    }
}


void serialPort::stop()
{
    this->isRunning = false;
}


void serialPort::startSend(int sendMode, float fdata)
{
    this->data = fdata;
    if(sendMode == 1) servoSend = true;
    if(sendMode == 2) adcSend = true;
}

// Public slots
void serialPort::CheckFrame()
{
    QByteArray rxData = serial->readAll();
    uint8_t rxbyte[11];
    memcpy(rxbyte, rxData, 11);

    if(rxbyte[0] == 0x02 && rxbyte[10] == 0x03 && this->check_checksum(&rxbyte[1], 8))
    {
        if(rxbyte[1] == 'S' && rxbyte[2] == 'A' && rxbyte[3] == 'D' && rxbyte[4] == 'C' )
        {
            float value;
            this->Bytes2float(&rxbyte[5], &value);
            if(value > 42)
            {
                value = pre_value;
            }
            pre_value = value;
            Q_EMIT readySensor(value);
        }

        else if(rxbyte[1] == 'S' && rxbyte[2] == 'C' && rxbyte[3] == 'O' && rxbyte[4] == 'N' )
        {
            flag_connect = true;
        }
    }
}


void serialPort::SendServoWidth()
{
    uint8_t sendbyte[11];
    this->ProtocolFrame(sendbyte, 1, this->data);
    ControlLib::delay_ms(20);
}


void serialPort::SendADCrequest()
{
    uint8_t sendbyte[11];
    this->ProtocolFrame(sendbyte, 2, this->data);
    ControlLib::delay_ms(20);
}

uint8_t serialPort::checksum(uint8_t *addr, uint8_t count)
{

    uint16_t sum = 0;
    for(int i=0; i<count; i++)
    {
        sum = sum + addr[i];
    }
    while (sum>>8)
    {
        sum = (sum & 0xFF) + (sum >> 8);
    }
    return(~((uint8_t)sum));
}

bool serialPort::check_checksum(uint8_t *addr, uint8_t count)
{
    uint16_t sum = 0;
  for(int i=0; i<(count+1); i++)
  {
    sum = sum + addr[i];
  }
  while (sum>>8)
    {
        sum = (sum & 0xFF) + (sum >> 8);
    }
  if(((uint8_t)sum)==0xFF) return true;
}


const int serialPort::GripperNutri = 100;
const int serialPort::GripperTH = 95;
const int serialPort::GripperWhiteCup = 82;
const int serialPort::GripperPinkCup = 85;
const int serialPort::GripperRelease = 0;
const int serialPort::GripperMax = 100;
