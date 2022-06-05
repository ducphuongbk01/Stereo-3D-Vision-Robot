#include "udp.h"
#include <QUdpSocket>
#include <QString>
#include <QByteArray>
#include <QThread>
#include <QTimer>
#include <QDebug>
#include <QObject>
#include <QString>
#include <QTime>
#include <QMessageBox>

udp::udp(QHostAddress h,quint16 p)
{
    this->_HostAddress = h;
    this->_port = p;
    this->client = new QUdpSocket;
    this->rx_buffer = new QByteArray;
    this->rx_joint = new QByteArray;
    this->rx_cart = new QByteArray;
}

udp::~udp()
{
    delete [] this->rx_buffer;
    delete [] this->rx_joint;
    delete [] this->rx_cart;
}

// Data Struct //
struct udp::TxData
{
    const char identifier[4] = {'Y','E','R','C'};
    const uint16_t header_size = 32;
    uint16_t data_size;
    const uint8_t reserve1 = 3;
    const uint8_t processing_division = 1;
    const uint8_t ack = 0;
    uint8_t id;
    const uint32_t block_no = 0;
    const char reserve2[8] = {'9','9','9','9','9','9','9','9'};
    uint16_t command_no;
    uint16_t instance;
    uint8_t attribute;
    uint8_t service;
    const uint16_t padding = 0;
};

struct udp::RxData
{
    char identifier[4];
    uint16_t header_size;
    uint16_t data_size;
    uint8_t reserve_1;
    uint8_t processing_division = 1;
    uint8_t ack;
    uint8_t id;
    uint32_t block_no;
    char reserve_2[8];
    uint8_t service;
    uint8_t status;
    uint8_t added_status_size;
    uint8_t padding;
    uint16_t added_status;
    uint16_t padding2;
};


struct udp::TxDataWritePosition
{
    const u_int32_t control_group_robot = 1;
    const u_int32_t control_group_station = 0;
    u_int32_t classification_in_speed = 0;
    u_int32_t speed;
    const uint32_t coordinate = 0x10;
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t rx;
    int32_t ry;
    int32_t rz;
    const u_int32_t reservation1 = 0;
    const u_int32_t reservation2 = 0;
    const u_int32_t type = 0;
    const u_int32_t expanded_type = 0;
    const u_int32_t tool_no = 0;
    const u_int32_t user_coordinate_no = 0;
    const u_int32_t base_1_position = 0;
    const u_int32_t base_2_position = 0;
    const u_int32_t base_3_position = 0;
    const u_int32_t station_1_position = 0;
    const u_int32_t station_2_position = 0;
    const u_int32_t station_3_position = 0;
    const u_int32_t station_4_position = 0;
    const u_int32_t station_5_position = 0;
    const u_int32_t station_6_position = 0;
};


struct udp::TxDataWritePulse
{
    const u_int32_t control_group_robot = 1;
    const u_int32_t control_group_station = 0;
    u_int32_t classification_in_speed = 0;
    u_int32_t speed;
    int32_t r1;
    int32_t r2;
    int32_t r3;
    int32_t r4;
    int32_t r5;
    int32_t r6;
    int32_t r7;
    int32_t r8;
    const u_int32_t tool_no = 0;
    const u_int32_t base_1_position = 0;
    const u_int32_t base_2_position = 0;
    const u_int32_t base_3_position = 0;
    const u_int32_t station_1_position = 0;
    const u_int32_t station_2_position = 0;
    const u_int32_t station_3_position = 0;
    const u_int32_t station_4_position = 0;
    const u_int32_t station_5_position = 0;
    const u_int32_t station_6_position = 0;
};

struct udp::TxDataWriteVariablePosition
{
    u_int32_t data_type;
    const u_int32_t figure = 0;
    const u_int32_t tool_no = 0;
    const u_int32_t user_coodirnate_no = 0;
    const u_int32_t extended_figure = 0;
    int32_t first_axis_position;
    int32_t second_axis_position;
    int32_t third_axis_position;
    int32_t fourth_axis_position;
    int32_t fifth_axis_position;
    int32_t sixth_axis_position;
    const int32_t seventh_axis_position = 0;
    const int32_t eighth_axis_position = 0;
};

// ///////////////////////
int32_t udp::ByteArray2Int32 (QByteArray* buffer,int start, int number)
{
    int32_t value = 0;
    for (int i = start;i < start+number; i++)
    {
        int32_t temp = (unsigned char) buffer->at(i) ;
        value += temp << 8*(i-start);
    }
    return value;
}

QString udp::ByteArray2Hex(QByteArray buffer)
{
    QString s;
    for (int i = 0; i < buffer.size(); i++)
    {
        QString c = QString::number(buffer.at(i),16);
        if(c.size()<2)
        {
            s.push_back('0');
            s.push_back(c);
        }
        else{
            s.push_back(c.at(c.size()-2));
            s.push_back(c.at(c.size()-1));
        }
        s.push_back(' ');
    }
    s = s.toUpper();
    return s;
}

//----------Get Postion Caresian/Pulse----------------------------------------------
// Thread
//----------------------------------------------------------------------------------
void udp:: run()
{
    while(this->isRunning)
    {
        if(this->readCart==true)
        {
            Q_EMIT Sig_ReadCart();
            usleep(100000);
            Q_EMIT Sig_ReadRobotCart();
        }

        if(this->readJoint==true)
        {
            Q_EMIT Sig_ReadJoint();
            usleep(100000);
            Q_EMIT Sig_ReadRobotJoint();
        }

        if(this->readPos==true)
        {
            Q_EMIT Sig_ReadCart();
            usleep(100000);
            Q_EMIT Sig_ReadJoint();
            usleep(100000);
            Q_EMIT Sig_ReadRobotPos();
        }

        if(this->readByte==true)
        {
            Q_EMIT Sig_ReadCart();
            usleep(100000);
            Q_EMIT Sig_ReadJoint();
            usleep(100000);
            Q_EMIT Sig_ReadByte();
            usleep(100000);
        }
    }
}

void udp::Stopthread()
{
    this->isRunning = false;
}

void udp::Startthread()
{
    this->isRunning = true;
}

bool udp::SendData(char* buffer, int lenght)
{
    this->client->writeDatagram(buffer,lenght,_HostAddress,_port);
    return 1;
}

QByteArray* udp::Get_rx_buffer()
{
    return this->rx_buffer;
}

bool udp::ConnectMotoman()
{
    bool ret = this->client->bind();
    this->isRunning = true;

    connect(this, SIGNAL(Sig_ReadCart()), this, SLOT(ReadCart()));
    connect(this, SIGNAL(Sig_ReadJoint()), this, SLOT(ReadJoint()));
    return ret;
}

bool udp::DisconnectMotoman ()
{
    disconnect(this, SIGNAL(Sig_ReadCart()), this, SLOT(ReadCart()));
    disconnect(this, SIGNAL(Sig_ReadJoint()), this, SLOT(ReadJoint()));
    this->isRunning = false;
    usleep(100000);
    this->exit(0);
    this->client->close();
    delete this->client;
    return true;
}

bool udp::OnServo()
{
    TxData sent_data;
    uint32_t data = 1;
    sent_data.id = RECEIVE_TYPE::ON_SERVO;
    sent_data.command_no = 0x83;
    sent_data.instance = 2;
    sent_data.attribute = 1;
    sent_data.service = 0x10;
    uint16_t data_length = sizeof (data);
    uint16_t total_length = 32 + data_length;
    sent_data.data_size = data_length;
    char* buffer = new char[total_length];
    memcpy(buffer,&sent_data,32);
    memcpy(buffer+32,&data,data_length);
    SendData(buffer,total_length);
    delete [] buffer;
    usleep(1000000);
    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 32)
    {
            RxData rxHeader;
            QByteArray rxdata;
            rxdata.resize(client->pendingDatagramSize());
            client->readDatagram(rxdata.data(),rxdata.size());
            memcpy(&rxHeader,rxdata.data(),32);
            if(rxHeader.status!=0)
            {
                return false;
            }
            return true;
    }
    else
    {
        return false;

    }
}
bool udp::OffServo()
{
    TxData sent_data;
    uint32_t data = 2;
    sent_data.id = RECEIVE_TYPE::OFF_SERVO;
    sent_data.command_no = 0x83;
    sent_data.instance = 2;
    sent_data.attribute = 1;
    sent_data.service = 0x10;
    uint16_t data_length = sizeof (data);
    uint16_t total_length = 32 + data_length;
    sent_data.data_size = data_length;
    char* buffer = new char[total_length];
    memcpy(buffer,&sent_data,32);
    memcpy(buffer+32,&data,data_length);
    SendData(buffer,total_length);
    delete [] buffer;

    usleep(1000000);
    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 32)
    {
            RxData rxHeader;
            QByteArray rxdata;
            rxdata.resize(client->pendingDatagramSize());
            client->readDatagram(rxdata.data(),rxdata.size());
            memcpy(&rxHeader,rxdata.data(),32);
            if(rxHeader.status!=0)
            {
                return false;
            }
            return true;
    }
    else
    {
        return false;
    }
}

bool udp::ReadCart()
{
    TxData sent_data;
    char* buffer = new char[sizeof (sent_data)];
    sent_data.id = RECEIVE_TYPE::GET_POSITION;
    sent_data.command_no = 0x75;
    sent_data.instance = 0x65;
    sent_data.attribute = 0;
    sent_data.service = 0x01;
    sent_data.data_size = 0;
    memcpy(buffer,&sent_data,sizeof (sent_data));
    SendData(buffer,sizeof (sent_data));
    delete [] buffer;

    usleep(50000);
    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 84 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(this->client->pendingDatagramSize() == 84)
    {
        RxData rxHeader;
        this->rx_cart->resize(this->client->pendingDatagramSize());
        this->client->readDatagram(this->rx_cart->data(),this->rx_cart->size());
        memcpy(&rxHeader,this->rx_cart->data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

QByteArray* udp::Get_CartPos()
{
    return this->rx_cart;
}

bool udp::ReadJoint()
{
    TxData sent_data;
    char* buffer = new char[sizeof (sent_data)];
    sent_data.id = RECEIVE_TYPE::GET_PULSE;
    sent_data.command_no = 0x75;
    sent_data.instance = 0x01;
    sent_data.attribute = 0;
    sent_data.service = 0x01;
    sent_data.data_size = 0;
    memcpy(buffer,&sent_data,sizeof (sent_data));
    SendData(buffer,sizeof (sent_data));
    delete [] buffer;

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 76 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 76)
    {
        RxData rxHeader;
        rx_joint->resize(client->pendingDatagramSize());
        client->readDatagram(rx_joint->data(),rx_joint->size());
        memcpy(&rxHeader,rx_joint->data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

QByteArray* udp::Get_PulsePos()
{
    return this->rx_joint;
}

bool udp::HomePos()
{
    TxData Header;
    Header.command_no = 0x8B;
    Header.instance = 0x01;
    Header.attribute = 1;
    Header.service = 0x02;
    Header.id = RECEIVE_TYPE::HOME_POS;

    TxDataWritePulse Pos;
    Pos.r1 = 0;
    Pos.r2 = 0;
    Pos.r3 = 0;
    Pos.r4 = 0;
    Pos.r5 = 0;
    Pos.r6 = 0;
    Pos.speed = 20*100;
    Header.data_size = sizeof (Pos);

    int total_length = sizeof (Header) + sizeof (Pos);
    char buffer[total_length];
    memcpy(buffer,&Header,sizeof (Header));
    memcpy(buffer + sizeof (Header),&Pos, total_length);
    SendData(buffer,sizeof (Header) + sizeof (Pos));
    return 1;
}

bool udp::SelectJob(char* jobname)
{
    uint32_t line_number=1;
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::SELECT_JOB;
    sent_data.command_no = 0x87;
    sent_data.instance = 1;
    sent_data.attribute = 0;
    sent_data.service = 0x02;
    uint16_t data_length = 36;
    uint16_t total_length = 32 + data_length;
    sent_data.data_size = data_length;
    char* buffer = new char[total_length];

    memcpy(buffer,&sent_data,32);
    memcpy(buffer+32,jobname,32);
    memcpy(buffer+32+32,&line_number,4);
    SendData(buffer,total_length);
    delete [] buffer;

    usleep(50000);

    if(client->pendingDatagramSize()>0)
    {
//        qDebug() << client->pendingDatagramSize();
        QByteArray rxdata;
        RxData rxHeader;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0) return false;
        return true;
    }
    else
    {
        return false;
    }

    return true;
}

bool udp::StartJob()
{
  TxData sent_data;
  sent_data.id = 9;
  sent_data.command_no = 0x86;
  sent_data.instance = 1;
  sent_data.attribute = 1;
  sent_data.service = 0x10;
  u_int32_t data = 1;
  sent_data.data_size = sizeof (data);
  char buffer [sizeof(sent_data)+ sizeof(data)];
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),&data,sizeof(data));
  SendData(buffer,sizeof(sent_data)+sizeof(data));
  usleep(1000000);

  this->timeoutTrigger = true;

  while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
  {
      QByteArray rxdata;
      rxdata.resize(client->pendingDatagramSize());
      client->readDatagram(rxdata.data(),rxdata.size());
      if(!this->timeoutTrigger) break;
  }

  if(client->pendingDatagramSize()==32)
  {
      QByteArray rxdata;
      RxData rxHeader;
      rxdata.resize(client->pendingDatagramSize());
      client->readDatagram(rxdata.data(),rxdata.size());
      memcpy(&rxHeader,rxdata.data(),32);
      if(rxHeader.status!=0) return false;
      return true;
  }
  else
  {
      return false;
  }
}


bool udp::WritePosCart(float speed,float X,float Y,float Z,float RX,float RY,float RZ)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::WRITE_POSITION;
    sent_data.command_no = 0x8A;
    sent_data.instance = 0x01;
    sent_data.attribute = 01;
    sent_data.service = 0x02;

    TxDataWritePosition position;
    position.x = X*1000;
    position.y = Y*1000;
    position.z = Z*1000;
    position.rx = RX*10000;
    position.ry = RY*10000;
    position.rz = RZ*10000;
    position.speed = speed*100;

    sent_data.data_size = sizeof(position);
    char* buffer = new char[sizeof(sent_data)+sizeof (position)];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof (sent_data),&position,sizeof (position));
    SendData(buffer,sizeof(sent_data)+sizeof (position));
    delete [] buffer;

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 32)
    {
        RxData rxHeader;
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        return true;
    }
    else
    {
        return false;
    }
}


bool udp::WritePosJoint(float speed,float R1,float R2,float R3,float R4,float R5,float R6)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::WRITE_PUSLE;
    sent_data.command_no = 0x8B;
    sent_data.instance = 0x01;
    sent_data.attribute = 01;
    sent_data.service = 0x02;

    TxDataWritePulse position;
    position.r1 = R1*(this->PULSE_PER_DEGREE_S);
    position.r2 = R2*(this->PULSE_PER_DEGREE_L);
    position.r3 = R3*(this->PULSE_PER_DEGREE_U);
    position.r4 = R4*(this->PULSE_PER_DEGREE_RBT);
    position.r5 = R5*(this->PULSE_PER_DEGREE_RBT);
    position.r6 = R6*(this->PULSE_PER_DEGREE_RBT);
    position.speed = speed*100;

    sent_data.data_size = sizeof(position);
    char* buffer = new char[sizeof(sent_data)+sizeof (position)];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof (sent_data),&position,sizeof (position));
    SendData(buffer,sizeof(sent_data)+sizeof (position));
    delete [] buffer;

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 32)
    {
        RxData rxHeader;
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool udp::GetCartasianPos(int32_t *cart)
{
    TxData sent_data;
    char* buffer = new char[sizeof (sent_data)];
    sent_data.id = RECEIVE_TYPE::GET_POSITION;
    sent_data.command_no = 0x75;
    sent_data.instance = 0x65;
    sent_data.attribute = 0;
    sent_data.service = 0x01;
    sent_data.data_size = 0;
    memcpy(buffer,&sent_data,sizeof (sent_data));
    SendData(buffer,sizeof (sent_data));
    delete [] buffer;

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 84 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 84)
    {
        QByteArray rxdata;
        RxData rxHeader;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)  return false;
        memcpy(cart,rxdata.data()+52,24);
        return true;
    }
    else
    {
        return false;
    }
}

bool udp::GetPulsePos(int32_t *pulse)
{
    TxData sent_data;
    char* buffer = new char[sizeof (sent_data)];
    sent_data.id = RECEIVE_TYPE::GET_PULSE;
    sent_data.command_no = 0x75;
    sent_data.instance = 0x01;
    sent_data.attribute = 0;
    sent_data.service = 0x01;
    sent_data.data_size = 0;
    memcpy(buffer,&sent_data,sizeof (sent_data));
    SendData(buffer,sizeof (sent_data));
    delete [] buffer;

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 76 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 76)
    {
        QByteArray rxdata;
        RxData rxHeader;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)  return false;
        memcpy(pulse,rxdata.data()+52,24);
        return true;
    }
    else
    {
        return false;
    }
}


bool udp::WriteMultipleVarCart(uint16_t instance, u_int32_t number, int32_t *pos)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::WRITE_POSITION;
    sent_data.command_no = 0x307;
    sent_data.instance = instance;
    sent_data.attribute = 0;
    sent_data.service = 0x34;

    char buffer[4+sizeof(sent_data)+ sizeof (TxDataWriteVariablePosition)*number];
    sent_data.data_size = sizeof(TxDataWriteVariablePosition)*number+4;

    for (int i = 0;i<number;i++)
    {
        TxDataWriteVariablePosition position;
        position.data_type = 0x11;
        position.first_axis_position = pos[6*i]*1000;
        position.second_axis_position = pos[6*i+1]*1000;
        position.third_axis_position = pos[6*i+2]*1000;
        position.fourth_axis_position = pos[6*i+3]*10000;
        position.fifth_axis_position = pos[6*i+4]*10000;
        position.sixth_axis_position = pos[6*i+5]*10000;
        memcpy(buffer+sizeof(sent_data)+sizeof(position)*i+4,&position,sizeof(position));
    }

    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof (sent_data),&number,sizeof (number));

    SendData(buffer,sent_data.header_size+sent_data.data_size);

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }


    if(client->pendingDatagramSize() == 32)
    {
        QByteArray rxdata;
        RxData rxHeader;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool udp::WriteMultipleVarJoint(uint16_t instance, u_int32_t number, int32_t *pos)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::WRITE_PUSLE;
    sent_data.command_no = 0x307;
    sent_data.instance = instance;
    sent_data.attribute = 0;
    sent_data.service = 0x34;

    char buffer[4+sizeof(sent_data)+ sizeof (TxDataWriteVariablePosition)*number];
    sent_data.data_size = sizeof(TxDataWriteVariablePosition)*number+4;

    for (int i = 0;i<number;i++)
    {
        TxDataWriteVariablePosition position;
        position.data_type = 0x00;
        position.first_axis_position = pos[6*i];
        position.second_axis_position = pos[6*i+1];
        position.third_axis_position = pos[6*i+2];
        position.fourth_axis_position = pos[6*i+3];
        position.fifth_axis_position = pos[6*i+4];
        position.sixth_axis_position = pos[6*i+5];
        memcpy(buffer+sizeof(sent_data)+sizeof(position)*i+4,&position,sizeof(position));
    }

    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof (sent_data),&number,sizeof (number));

    SendData(buffer,sent_data.header_size+sent_data.data_size);

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }


    if(client->pendingDatagramSize() == 32)
    {
        QByteArray rxdata;
        RxData rxHeader;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool udp::WriteVarCart(uint16_t instance, int32_t *pos)
{
    TxData sent_data;
    sent_data.id = 0x0D;
    sent_data.command_no = 0x7F;
    sent_data.instance = instance;
    sent_data.attribute = 0;
    sent_data.service = 0x02;
    sent_data.data_size = 0;

    TxDataWriteVariablePosition position;
    position.data_type = 0x11;
    position.first_axis_position = pos[0]*1000;
    position.second_axis_position = pos[1]*1000;
    position.third_axis_position = pos[2]*1000;
    position.fourth_axis_position = pos[3]*10000;
    position.fifth_axis_position = pos[4]*10000;
    position.sixth_axis_position = pos[5]*10000;

    sent_data.data_size = sizeof(position);
    char* buffer = new char[sizeof(sent_data)+sizeof (position)];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof (sent_data),&position,sizeof (position));
    SendData(buffer,sizeof(sent_data)+sizeof (position));
    delete [] buffer;

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 32)
    {
        QByteArray rxdata;
        RxData rxHeader;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool udp::WriteVarJoint(uint16_t instance, float *pos)
{
    TxData sent_data;
    sent_data.id = 0x07;
    sent_data.command_no = 0x7F;
    sent_data.instance = instance;
    sent_data.attribute = 0;
    sent_data.service = 0x02;
    sent_data.data_size = 0;

    TxDataWriteVariablePosition position;
    position.data_type = 0x00;
    position.first_axis_position = pos[0]*PULSE_PER_DEGREE_S;
    position.second_axis_position = pos[1]*PULSE_PER_DEGREE_L;
    position.third_axis_position = pos[2]*PULSE_PER_DEGREE_U;
    position.fourth_axis_position = pos[3]*PULSE_PER_DEGREE_RBT;
    position.fifth_axis_position = pos[4]*PULSE_PER_DEGREE_RBT;
    position.sixth_axis_position = pos[5]*PULSE_PER_DEGREE_RBT;

    sent_data.data_size = sizeof(position);
    char* buffer = new char[sizeof(sent_data)+sizeof (position)];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof (sent_data),&position,sizeof (position));
    SendData(buffer,sizeof(sent_data)+sizeof (position));
    delete [] buffer;

    usleep(50000);

    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 32)
    {
        QByteArray rxdata;
        RxData rxHeader;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool udp::ReadIO(uint16_t instance, int8_t *IO_status)
{
    TxData sent_data;
    uint32_t data = 0;
    sent_data.id = RECEIVE_TYPE::READ_IO;
    sent_data.command_no = 0x78;
    sent_data.instance = instance;
    sent_data.attribute = 1;
    sent_data.service = 0x0E;

    uint16_t data_length = sizeof (data);
    uint16_t total_length = 32 + data_length;
    sent_data.data_size = data_length;
    char* buffer = new char[total_length];
    memcpy(buffer,&sent_data,32);
    memcpy(buffer+32,&data,data_length);
    SendData(buffer,total_length);
    delete [] buffer;

    usleep(50000);
//    this->timeoutTrigger = true;

//    while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
//    {
//        QByteArray rxdata;
//        rxdata.resize(client->pendingDatagramSize());
//        client->readDatagram(rxdata.data(),rxdata.size());
//        if(!this->timeoutTrigger) break;
//    }

    if(client->pendingDatagramSize() > 32)
    {
            RxData rxHeader;
            QByteArray rxdata;
            rxdata.resize(client->pendingDatagramSize());
            client->readDatagram(rxdata.data(),rxdata.size());
            memcpy(&rxHeader,rxdata.data(),32);
            if(rxHeader.status!=0)
            {
                return false;
            }
            memcpy(IO_status,rxdata.data()+32,1);
            return true;
    }
    else
    {
        return false;
    }
}

bool udp::ReadByte(uint16_t instance, int8_t *Byte)
{
    TxData sent_data;
    uint32_t data = 0;
    sent_data.id = RECEIVE_TYPE::READ_BYTE;
    sent_data.command_no = 0x7A;
    sent_data.instance = instance;
    sent_data.attribute = 1;
    sent_data.service = 0x0E;

    uint16_t data_length = sizeof (data);
    uint16_t total_length = 32 + data_length;
    sent_data.data_size = data_length;
    char* buffer = new char[total_length];
    memcpy(buffer,&sent_data,32);
    memcpy(buffer+32,&data,data_length);
    SendData(buffer,total_length);
    delete [] buffer;

    usleep(50000);
//    this->timeoutTrigger = true;

//    while(client->pendingDatagramSize() != 33 && client->pendingDatagramSize()>0)
//    {
//        QByteArray rxdata;
//        rxdata.resize(client->pendingDatagramSize());
//        client->readDatagram(rxdata.data(),rxdata.size());
//        if(!this->timeoutTrigger) break;
//    }

    if(client->pendingDatagramSize() == 33)
    {
            RxData rxHeader;
            QByteArray rxdata;
            rxdata.resize(client->pendingDatagramSize());
            client->readDatagram(rxdata.data(),rxdata.size());
            memcpy(&rxHeader,rxdata.data(),32);
            if(rxHeader.status!=0)
            {
                return false;
            }
            memcpy(Byte,rxdata.data()+32,1);
            return true;
    }
    else
    {
        return false;
    }
}

bool udp::WriteByte(u_int16_t instance, uint8_t data)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::WRITE_BYTE;
    sent_data.command_no = 0x7A;
    sent_data.instance = instance;
    sent_data.attribute = 1;
    sent_data.service = 0x10;
    sent_data.data_size = sizeof (data);

    char buffer [sizeof(sent_data)+ sizeof(data)];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof(sent_data),&data,sizeof(data));
    SendData(buffer,sizeof(sent_data)+sizeof(data));

    usleep(50000);
//  this->timeoutTrigger = true;

//  while(client->pendingDatagramSize() != 32 && client->pendingDatagramSize()>0)
//  {
//      QByteArray rxdata;
//      rxdata.resize(client->pendingDatagramSize());
//      client->readDatagram(rxdata.data(),rxdata.size());
//      if(!this->timeoutTrigger) break;
//  }

//  if(client->pendingDatagramSize() == 32)
//  {
//      QByteArray rxdata;
//      RxData rxHeader;
//      rxdata.resize(client->pendingDatagramSize());
//      client->readDatagram(rxdata.data(),rxdata.size());
//      memcpy(&rxHeader,rxdata.data(),32);
//      if(rxHeader.status!=0)
//      {
//          return false;
//      }
//      return true;
//  }
//  else
//  {
//      return false;
//  }
    return true;
}

bool udp::ReadMultipleBytes(u_int16_t instance, uint32_t number, uint8_t *data)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::READ_BYTE;
    sent_data.command_no = 0x302;
    sent_data.instance = instance;
    sent_data.attribute = 0;
    sent_data.service = 0x33;
    sent_data.data_size = 4;
    char buffer [sizeof(sent_data)+ 4];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof(sent_data),&number,4);

    SendData(buffer,sent_data.header_size+sent_data.data_size);
    usleep(50000);
    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 36+number && client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 36+number)
    {
        QByteArray array;
        RxData rxHeader;
        array.resize(client->pendingDatagramSize());
        client->readDatagram(array.data(),array.size());
        memcpy(&rxHeader,array.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        memcpy(data,array.data()+36,number);
        return true;
    }
    else
    {
        return false;
    }
}

bool udp::WriteMultipleBytes(u_int16_t instance, uint32_t number, uint8_t *data)
{
    TxData sent_data;
    sent_data.id = RECEIVE_TYPE::WRITE_BYTE;
    sent_data.command_no = 0x302;
    sent_data.instance = instance;
    sent_data.attribute = 0;
    sent_data.service = 0x34;
    sent_data.data_size = 4 + number;
    char buffer [sizeof(sent_data) + 4 + number];
    memcpy(buffer,&sent_data,sizeof (sent_data));
    memcpy(buffer+sizeof(sent_data),&number,4);
    memcpy(buffer+sizeof(sent_data)+4,data,number);

    SendData(buffer,sent_data.header_size+sent_data.data_size);
    usleep(50000);
    if(client->pendingDatagramSize()>0)
    {
        QByteArray rxdata;
        RxData rxHeader;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool udp::ReadStatus(int32_t *Byte)
{
    TxData sent_data;

    sent_data.id = RECEIVE_TYPE::ON_SERVO;
    sent_data.command_no = 0x72;
    sent_data.instance = 1;
    sent_data.attribute = 0x02;
    sent_data.service = 0x0E;

    uint16_t data_length = 0;
    uint16_t total_length = 32 + data_length;
    sent_data.data_size = data_length;
    char* buffer = new char[total_length];
    memcpy(buffer,&sent_data,32);
    SendData(buffer,total_length);
    delete [] buffer;
    usleep(100000);
    this->timeoutTrigger = true;

    while(client->pendingDatagramSize() != 36 && client->pendingDatagramSize() > 0)
    {
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        if(!this->timeoutTrigger) break;
    }

    if(client->pendingDatagramSize() == 36)
    {
        RxData rxHeader;
        QByteArray rxdata;
        rxdata.resize(client->pendingDatagramSize());
        client->readDatagram(rxdata.data(),rxdata.size());
        memcpy(&rxHeader,rxdata.data(),32);
        if(rxHeader.status!=0)
        {
            return false;
        }
        memcpy(Byte,rxdata.data()+32,4);
        return true;
    }
    else
    {
        return false;

    }
}
const double udp::PULSE_PER_DEGREE_S    = 34816/30;
const double udp::PULSE_PER_DEGREE_L    = 102400/90;
const double udp::PULSE_PER_DEGREE_U    = 51200/90;
const double udp::PULSE_PER_DEGREE_RBT  = 10204/30;

const float udp::Home_2[6] = {0,-18.9094,-40.493,0,113.824,47.0588};

const float udp::Home_1[6] = {-86.6379,-18.9094,-40.493,0,113.824,47.0588};
