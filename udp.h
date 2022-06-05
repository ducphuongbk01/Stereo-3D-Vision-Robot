#ifndef UDP_H
#define UDP_H

#include <QUdpSocket>
#include <QThread>
#include <QTimer>
#include <QCoreApplication>


class udp : public QThread
{
    Q_OBJECT
public:
    udp(QHostAddress h,quint16 p);
    ~udp();

    enum RECEIVE_TYPE  {ON_SERVO =      0x00,
                        OFF_SERVO =     0x01,
                        GET_POSITION =  0x02,
                        GET_PULSE =     0x03,
                        WRITE_POSITION = 0x04,
                        WRITE_PUSLE =   0x05,
                        SELECT_JOB =    0x06,
                        START_JOB =     0x07,
                        HOME_POS =      0x08,
                        SAVE_FILE =     0x09,
                        LOAD_FILE =     0x0A,
                        DELETE_FILE =   0x0B,
                        GET_VARPOS =    0x0C,
                        WRITE_VARPOS =  0x0D,
                        WRITE_BYTE =    0x0E,
                        READ_BYTE =     0x0F,
                        READ_IO =       0x10};

    QUdpSocket* client;
    bool timeoutTrigger = false;

    static const double PULSE_PER_DEGREE_S;
    static const double PULSE_PER_DEGREE_L;
    static const double PULSE_PER_DEGREE_U;
    static const double PULSE_PER_DEGREE_RBT;

    static const float Home_1[6];

    static const float Home_2[6];

    // Function //


    // Transmit data //

        bool SendData(char* buffer, int lenght);

        void Startthread();

        void Stopthread();

        bool ConnectMotoman();

        bool DisconnectMotoman();

        bool OnServo();

        bool OffServo();

        bool ReadStatus(int32_t *Byte);

        bool HomePos();

        bool SelectJob(char* jobname);

        bool StartJob();

        bool WritePosCart(float speed,float X,float Y,float Z,float RX,float RY,float RZ);

        bool WritePosJoint(float speed,float R1,float R2,float R3,float R4,float R5,float R6);

        bool WriteMultipleVarCart(uint16_t instance, u_int32_t number, int32_t *pos);

        bool WriteMultipleVarJoint(uint16_t instance, u_int32_t number, int32_t *pos);

        bool WriteVarCart(uint16_t instance, int32_t *pos);

        bool WriteVarJoint(uint16_t instance, float *pos);

        bool GetCartasianPos(int32_t *cart);

        bool GetPulsePos(int32_t *pulse);

        bool ReadIO(uint16_t instance,int8_t *IO_status);

        bool ReadByte(uint16_t instance, int8_t *Byte);

        bool WriteByte(uint16_t instance,uint8_t data);

        bool ReadMultipleBytes(u_int16_t instance, uint32_t number, uint8_t *data);

        bool WriteMultipleBytes(u_int16_t instance, uint32_t number, uint8_t *data);


        // Convert Function //
        QByteArray Hex2ByteArray (QString s);
        QString ByteArray2Hex(QByteArray buffer);
        int32_t ByteArray2Int32 (QByteArray* buffer,int start, int number);

        QByteArray *Get_rx_buffer();
        QByteArray *Get_CartPos();
        QByteArray *Get_PulsePos();


        QString rx_data;
        bool readPos;
        bool readCart;
        bool readJoint;
        bool readByte;
        bool isFinished = true;

public slots:
    void OFFTrigger()
    {
        this->timeoutTrigger = false;
    };

    bool ReadCart();

    bool ReadJoint();

protected:
    void run();
//     static const QString ON_SERVO_CMD;
//     static const QString OFF_SERVO_CMD;
//     static const QString GET_POS_CMD;
//     static const QString GET_POS_PULSE;
//     static const QString HOME_POSITION;
//     static const QString WRITE_POS_HEADER;

private:
    QHostAddress _HostAddress;
    quint16 _port;
    QByteArray *rx_buffer;
    QByteArray *rx_bufferFile;

    QByteArray *rx_cart;
    QByteArray *rx_joint;

    QByteArray WritePos;
    bool isRunning;

    // Sturct //
    struct TxData;
    struct RxData;
    struct TxDataWritePosition;
    struct TxDataWritePulse;
    struct TxDataWriteVariablePosition;

signals:
    void Sig_ReadRobotPos();
    void Sig_ReadRobotCart();
    void Sig_ReadRobotJoint();

    void Sig_ReadCart();
    void Sig_ReadJoint();
    void Sig_ReadPos();
    void Sig_ReadByte();
};

#endif // UDP_H
