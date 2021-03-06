#include "process.h"
#include <math.h>

float sumlevel = 0.0;
int32_t Cart[6];

process::process(int Mode, udp *CtrlSocket, serialPort *Stm, RealsenseCamera *camera,
                 std::string calib_path, std::string bottle_path, std::string cup_path)
{
    this->processMode = Mode;
    this->processSocket = CtrlSocket;
    this->processStm = Stm;
    this->camera = camera;

    full_water = 30.0;

    connect(this->processStm, SIGNAL(readySensor(float)), this, SLOT(Waterlvl(float)));
    connect(this, SIGNAL(Sig_JOBGapChai()), this, SLOT(JOB_GapChai()));
    connect(this, SIGNAL(Sig_JOBTraChai()), this, SLOT(JOB_TraChai()));
    connect(this, SIGNAL(Sig_JOBGapLy()), this, SLOT(JOB_GapLy()));
    connect(this, SIGNAL(Sig_JOBTraLy()), this, SLOT(JOB_TraLy()));
    connect(this, SIGNAL(Sig_JOBDomucnuoc()), this, SLOT(JOB_Domucnuoc()));
    connect(this, SIGNAL(Sig_JOBRotnuoc()), this, SLOT(JOB_Rotnuoc()));
    connect(this, SIGNAL(Sig_JOBClear(int)), this, SLOT(JOB_Clear(int)));
    connect(this, SIGNAL(Sig_JOBMain()), this, SLOT(JOB_Main()));



    Convert::loadMatFile(this->c2b, calib_path + "/calibw2c.yml");
    Convert::loadMatFile(this->g2e, calib_path + "/calibo2e.yml");

//    ppf = new PoseEstimation(bottle_path,
//                             0.025, 0.05, 30,
//                             1.0/30.0, 0.03, 7, 100, 0.005, 2.5, 5);

    ppf = new PoseEstimation(cup_path,
                             0.025, 0.05, 30,
                             1.0/30.0, 0.03, 7, 100, 0.005, 2.5, 5);
    connect(this->camera, SIGNAL(singlePCReady()), this, SLOT(caculate()));

    ControlLib::delay_ms(500);
    Q_EMIT Sig_JOBClear(2);
    ControlLib::delay_ms(100);
    ppf->start();
}

void process::startThread()
{
    bStart = true;
}

void process::stopThread()
{
    bStart = false;
}

void process::run()
{
    bStart = true;

    while(bStart)
    {
        // Main JOB
        if(!bStartProcess) continue;

        this->camera->stopBB();

        Q_EMIT Sig_JOBMain();
        ControlLib::delay_ms(200);

        while(!bMain)
        {
            ControlLib::delay_ms(10);
        };

        bMain = false;

        ControlLib::Update_Pos(this->processMode,false,this->processSocket);
        ControlLib::delay_ms(100);
        this->processSocket->WriteByte(20, 1);
        ControlLib::delay_ms(50);
        ControlLib::Update_Pos(this->processMode,true,this->processSocket);

        // Gap Ly
        Q_EMIT Sig_JOBGapLy();
        ControlLib::delay_ms(200);

        while(!bFinishGapLy)
        {
            ControlLib::delay_ms(10);
        };

        bFinishGapLy = false;

        Q_EMIT Sig_JOBDomucnuoc();
        ControlLib::delay_ms(200);

        while(!bGapChai && !bFullWater)
        {
            ControlLib::delay_ms(10);
        };


        if(bGapChai)
        {
            bGapChai = false;
            bFullWater = false;

            // Gap Chai
            Q_EMIT Sig_JOBGapChai();
            ControlLib::delay_ms(200);

            while(!bRotnuoc)
            {
                ControlLib::delay_ms(10);
            };

            bRotnuoc = false;

            // Rot nuoc
            Q_EMIT Sig_JOBRotnuoc();
            ControlLib::delay_ms(200);


            while(!bTraChai)
            {
                ControlLib::delay_ms(10);
            };

            bTraChai = false;


            // Tra chai
            Q_EMIT Sig_JOBTraChai();
            ControlLib::delay_ms(200);
        }

        bGapChai = false;
        bFullWater = false;


        while(!bTraLy)
        {
            ControlLib::delay_ms(10);
        };

        bTraLy = false;

        // Tra Ly
        Q_EMIT Sig_JOBTraLy();
        ControlLib::delay_ms(200);

        while(!bClear)
        {
            ControlLib::delay_ms(10);
        };

        bClear = false;

        Q_EMIT Sig_JOBClear(1);
        ControlLib::delay_ms(200);

        bStartProcess = false;

        Q_EMIT Sig_finishProcess(true);

        this->camera->startBB();

    }
}

void process::JOB_Main()
{
    ControlLib::Update_Pos(this->processMode,false,this->processSocket);
    ControlLib::delay_ms(125);
    QString Job = "LV-DP-MAIN";
    char jobname[Job.length()];
    strcpy(jobname,Job.toStdString().c_str());
    this->processSocket->SelectJob(jobname);
    ControlLib::delay_ms(100);
    this->processSocket->StartJob();
    ControlLib::delay_ms(100);
    ControlLib::Update_Pos(this->processMode,true,this->processSocket);

    bMain = true;
}

void process::JOB_GapChai()
{
    // Finish JOB_GapLy and Start JOB_GapChai
    this->processSocket->WriteByte(21, 0);
    ControlLib::delay_ms(100);
    int8_t byte;
    this->processSocket->ReadByte(50, &byte);

    while(byte != 1)
    {
        this->processSocket->ReadByte(50, &byte);
        ControlLib::delay_ms(100);
    }
    this->processStm->startSend(1,serialPort::GripperNutri);
    ControlLib::delay_ms(500);

    this->processSocket->WriteByte(70, 1);
    ControlLib::delay_ms(100);

    // Finish JOB_GapChai and Start JOtrueB_RotNuoc
    int8_t byte12;
    this->processSocket->ReadByte(12, &byte12);
    ControlLib::delay_ms(100);

    while(byte12 != 1)
    {
        this->processSocket->ReadByte(12, &byte12);
        ControlLib::delay_ms(100);
    }

    bRotnuoc = true;
}

void process::JOB_TraChai()
{
    // Start JOB_TraChai
    this->processSocket->WriteByte(13, 0);
    ControlLib::delay_ms(100);

    int8_t byte;
    this->processSocket->ReadByte(50, &byte);

    while(byte != 0)
    {
        this->processSocket->ReadByte(50, &byte);
        ControlLib::delay_ms(100);
    }

    this->processStm->startSend(1,serialPort::GripperRelease);
    ControlLib::delay_ms(1000);

    this->processSocket->WriteByte(70, 0);
    ControlLib::delay_ms(100);

    // Finish JOB_TraChai and Start JOB_TraLy
    int8_t byte14;
    this->processSocket->ReadByte(14, &byte14);
    ControlLib::delay_ms(100);

    while(byte14 != 1)
    {
        this->processSocket->ReadByte(14, &byte14);
        ControlLib::delay_ms(100);
    }

    bTraLy = true;
}

void process::JOB_GapLy()
{
    // Gap Ly
    int8_t byte;
    this->processSocket->ReadByte(50, &byte);

    while(byte != 1)
    {
        this->processSocket->ReadByte(50, &byte);
        ControlLib::delay_ms(100);
    }

    this->processStm->startSend(1,serialPort::GripperPinkCup);
    ControlLib::delay_ms(500);

    this->processSocket->GetCartasianPos(Cart);
    ControlLib::delay_ms(100);

    this->processSocket->WriteByte(70, 1);
    ControlLib::delay_ms(100);

    // Tha ly
    this->processSocket->ReadByte(50, &byte);

    while(byte != 0)
    {
        this->processSocket->ReadByte(50, &byte);
        ControlLib::delay_ms(100);
    }

    this->processStm->startSend(1,serialPort::GripperRelease);
    ControlLib::delay_ms(500);

    this->processStm->startSend(2,1000);

    this->processSocket->WriteByte(70, 0);
    ControlLib::delay_ms(100);


    // hoan thanh viec rot nuoc
    int8_t byte2;
    this->processSocket->ReadByte(11, &byte2);
    ControlLib::delay_ms(100);

    while(byte2 != 1)
    {
        this->processSocket->ReadByte(11, &byte2);
        ControlLib::delay_ms(100);
    }

    bFinishGapLy = true;

}

void process::JOB_TraLy()
{
    // Start JOB_TraLy
    if(Cart[3]/10000 > 100)
    {
        int32_t pos[12] = {Cart[0]/1000,Cart[1]/1000,52,87,-43,80,
                           Cart[0]/1000,Cart[1]/1000,-11,87,-43,80};

        ControlLib::Update_Pos(processMode,false,this->processSocket);
        ControlLib::delay_ms(125);
        processSocket->WriteMultipleVarCart(100,2,pos);
        ControlLib::Update_Pos(processMode,true,this->processSocket);
    }
    else if(Cart[3]/10000 > 92 || Cart[3]/10000 < 80)
    {
        int32_t pos[6] = {Cart[0]/1000,Cart[1]/1000,Cart[2]/1000,87,Cart[4]/10000,Cart[5]/10000};
        ControlLib::Update_Pos(processMode,false,this->processSocket);
        ControlLib::delay_ms(125);
        processSocket->WriteMultipleVarCart(101,1,pos);
        ControlLib::Update_Pos(processMode,true,this->processSocket);
    }

    this->processSocket->WriteByte(24, 10);
    ControlLib::delay_ms(100);

    int8_t byte;
    this->processSocket->ReadByte(50, &byte);

    while(byte != 1)
    {
        this->processSocket->ReadByte(50, &byte);
        ControlLib::delay_ms(100);
    }

    this->processStm->startSend(1,serialPort::GripperPinkCup);
    ControlLib::delay_ms(500);

    this->processSocket->WriteByte(70, 1);
    ControlLib::delay_ms(100);

    int8_t byte2;
    this->processSocket->ReadByte(50, &byte2);

    while(byte2 != 0)
    {
        this->processSocket->ReadByte(50, &byte2);
        ControlLib::delay_ms(100);
    }

    this->processStm->startSend(1,serialPort::GripperRelease);
    ControlLib::delay_ms(500);

    this->processSocket->WriteByte(70, 0);
    ControlLib::delay_ms(100);

    // Finish JOB_TraLy and Start JOB_Clear
    int8_t byte15;
    this->processSocket->ReadByte(15, &byte15);
    ControlLib::delay_ms(100);

    while(byte15 != 1)
    {
        this->processSocket->ReadByte(15, &byte15);
        ControlLib::delay_ms(100);
    }

    bClear = true;
}

void process::JOB_Rotnuoc()
{
    // Start JOB_Rotnuoc
    this->processSocket->WriteByte(12, 0);
    ControlLib::delay_ms(100);

    float lvl1, lvl2;
    float elf = 0.75;
    int alfa = 0;
    int tim = 2000;
    this->processStm->startSend(2,1000);
    ControlLib::delay_ms(100);
    lvl1 = lvl;
    this->processStm->startSend(2,-1000);
    int k = 0;
    int n = 0;

    while(lvl1 < 28)
    {
        this->processStm->startSend(2,1000);
        ControlLib::delay_ms(100);
        lvl1 = lvl;
        this->processStm->startSend(2,-1000);
        n++;
        if(n>=10) break;
    }

    while(true)
    {
        ControlLib::delay_ms(500);
        this->processStm->startSend(2,1000);
        ControlLib::delay_ms(100);
        lvl2 = lvl;
        this->processStm->startSend(2,-1000);

        while(lvl2<24)
        {
            this->processStm->startSend(2,1000);
            ControlLib::delay_ms(100);
            lvl2 = lvl;
            k++;
            if(k > 10) break;
        }

        if(lvl2 < full_water)
        {
            float lvl_temp = 0.0;
            ControlLib::delay_ms(300);
            this->processStm->startSend(2,1000);
            ControlLib::delay_ms(100);
            this->processStm->startSend(2,-1000);
            lvl_temp = lvl;
            if(lvl_temp < full_water)
            {
                qDebug() << "Break";
                break; // day
            }
            else
            {
                continue;
            }
        }
        else
        {
            if(lvl1 - lvl2 > elf)
            {
                lvl1 = lvl2;
                continue;
            }
            else
            {
                if(alfa == 6)
                {
                    Q_EMIT Sig_OOW();

                    break;
                }
                else
                {
                    alfa++;
                    ControlLib::delay_ms(tim);
                    ControlLib::Update_Pos(this->processMode,false,this->processSocket);
                    ControlLib::delay_ms(125);
                    this->processSocket->WriteByte(71, alfa);
                    ControlLib::delay_ms(500);
                    lvl1 = lvl2;
                    ControlLib::Update_Pos(this->processMode,true,this->processSocket);
                }
            }
        }
    }
    this->processSocket->WriteByte(71, 7);
    ControlLib::delay_ms(500);

    // Finish JOB_Rotnuoc and Start JOB_TraChai
    int8_t byte13;
    this->processSocket->ReadByte(13, &byte13);
    ControlLib::delay_ms(100);
    while(byte13 != 1)
    {
        this->processSocket->ReadByte(13, &byte13);
        ControlLib::delay_ms(100);
    }

    bTraChai = true;
}

void process::JOB_Domucnuoc()
{
    int8_t byte;
    int n = 0;
    this->processSocket->WriteByte(11, 0);
    ControlLib::delay_ms(100);
    // Do muc nuoc
    float lvl_temp = 0.0;
    this->processStm->startSend(2,1000);
    ControlLib::delay_ms(100);
    lvl_temp = lvl;
    this->processStm->startSend(2,-1000);

    while(lvl_temp < 28)
    {
        this->processStm->startSend(2,1000);
        ControlLib::delay_ms(100);
        lvl_temp = lvl;
        this->processStm->startSend(2,-1000);
        n++;
        if(n>=3) break;
    }

    if(lvl_temp < 34)
    {
        lvl_temp = 0.0;
        this->processStm->startSend(2,1000);
        ControlLib::delay_ms(100);
        this->processStm->startSend(2,-1000);
        lvl_temp = lvl;
        if(lvl_temp < 33)
        {
            qDebug() << lvl_temp;
            // hoan thanh viec rot nuoc

            this->processSocket->WriteByte(72, 33);
            ControlLib::delay_ms(100);

            // Hoan thanh do muc nuoc
            this->processSocket->WriteByte(30, 1);
            ControlLib::delay_ms(100);

            this->processSocket->ReadByte(21, &byte);
            ControlLib::delay_ms(100);

            while(byte != 1)
            {
                this->processSocket->ReadByte(21, &byte);
                ControlLib::delay_ms(100);
            }
            bTraLy = true;
            bFullWater = true;
            return;
        }
    }

    this->processSocket->WriteByte(72, 10);
    ControlLib::delay_ms(100);

    // Hoan thanh do muc nuoc
    this->processSocket->WriteByte(30, 1);
    ControlLib::delay_ms(100);

    this->processSocket->ReadByte(21, &byte);
    ControlLib::delay_ms(100);

    while(byte != 1)
    {
        this->processSocket->ReadByte(21, &byte);
        ControlLib::delay_ms(100);
    }

    bGapChai = true;
}

void process::JOB_Clear(int mode)
{
    bGapChai=false;
    bTraChai=false;
    bGapLy=false;
    bTraLy=false;
    bRotnuoc=false;
    bFullWater= false;
    bClear=false;
    bMain=false;
    bFinishGapLy=false;

    if(mode == 1)
    {
        // Start JOB_Clear
        ControlLib::Update_Pos(this->processMode,false,this->processSocket);
        this->processSocket->WriteByte(15, 0);
        ControlLib::delay_ms(100);
        ControlLib::Update_Pos(this->processMode,true,this->processSocket);
    }
    else if (mode == 2)
    {
        ControlLib::Update_Pos(this->processMode,false,this->processSocket);
        QString Job = "LV-DP-CLEAR";
        char jobname[Job.length()];
        strcpy(jobname,Job.toStdString().c_str());
        this->processSocket->SelectJob(jobname);
        ControlLib::delay_ms(500);
        this->processSocket->StartJob();
        ControlLib::delay_ms(200);
        ControlLib::Update_Pos(this->processMode,true,this->processSocket);
    }
}

void process::IpruptTimer()
{

}

void process::Waterlvl(float level)
{
    lvl = level;
}

void process::Startprocess()
{
    this->camera->startProcess();
}

void process::caculate()
{
    if(!bStartProcess)
    {
        this->camera->stopProcess();

        ControlLib::delay_ms(100);

        cv::ppf_match_3d::Pose3DPtr result;
        cv::Mat pcResult;
        std::vector<std::vector<int32_t>> pulses;

        bool ret1 = this->ppf->estimatePose(this->camera->pcCup_Mat, result, pcResult);

        if(ret1)
        {
            bool ret2 = this->ppf->caculateUsedPulsePose(result, this->c2b, this->g2e, pulses);
            //write
//            if(ret2 && abs(pulses.at(0).at(5))<17401 && abs(pulses.at(1).at(5))<17401)
            if(ret2)
            {
                int32_t pulseWrite[12];
                for(int i=0; i<6; i++)
                {
                    pulseWrite[i] = pulses.at(0).at(i);
                    pulseWrite[i+6] = pulses.at(1).at(i);
                }

                Q_EMIT finish(pulseWrite);

                ControlLib::Update_Pos(this->processMode,false,this->processSocket);
                ControlLib::delay_ms(30);
                this->processSocket->WriteMultipleVarJoint(100, 2, pulseWrite);
                ControlLib::delay_ms(100);
                this->bStartProcess = true;
                ControlLib::AppOutput("Processing",this->ProcessTxt);
            }
            else
            {
                ControlLib::AppOutput("Cannot move to caculated position.",this->ProcessTxt);
                this->bStartProcess = false;
                Q_EMIT Sig_finishProcess(true);
                Q_EMIT Sig_poseERROR();
            }
        }
        else
        {
            ControlLib::AppOutput("There is no pose to start process.",this->ProcessTxt);
            this->bStartProcess = false;
//            this->camera->startProcess();
            Q_EMIT Sig_finishProcess(true);
            Q_EMIT Sig_poseERROR();
        }
    }
}
