#include "calibcolor.h"
#include "ui_calibcolor.h"
#include <QDebug>
#include <QFileDialog>
#include <QDir>
#include <QString>

CalibColor::CalibColor(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CalibColor)
{
    ui->setupUi(this);
    setWindowTitle("Calibration Color Form");
    this->parent = parent;

    this->ui->hsIterationClosingValue->setMinimum(1);

    this->ui->lbKernelBlurSize->setText("1");
    this->ui->lbKernelClosingSize->setText("1");
    this->ui->lbIterationClosingValue->setText("1");

    connect(this->ui->hsKernelBlurSize, SIGNAL(valueChanged(int)), this, SLOT(changeLabelBlurKernelSize(int)));
    connect(this->ui->hsKernelClosingSize, SIGNAL(valueChanged(int)), this, SLOT(changeLabelClosingKernelSize(int)));

    this->calib = new ColorCalibration(640, 480, "/home/phuongdoan/Code/ThesisGUI/config.json",
                                                 this->ui->hsKernelBlurSize,
                                                 this->ui->hsThresholdValue,
                                                 this->ui->hsKernelClosingSize,
                                                 this->ui->hsIterationClosingValue);
    connect(this->calib, SIGNAL(frameReady(QImage,QImage)), this, SLOT(streamShow(QImage,QImage)));
    connect(this->calib, SIGNAL(refFrameReady(QImage)), this, SLOT(refFrameShow(QImage)));
}

CalibColor::~CalibColor()
{
//    this->calib->terminate();
    delete ui;
}

void CalibColor::streamShow(QImage filter, QImage mask)
{
    ControlLib::ShowImage(this->ui->streamFilter, filter, calib->width, calib->height);
    ControlLib::ShowImage(this->ui->streamMask, mask, calib->width, calib->height);
}

void CalibColor::refFrameShow(QImage refFrame)
{
    ControlLib::ShowImage(this->ui->imageRef, refFrame, calib->width, calib->height);
}

void CalibColor::changeLabelBlurKernelSize(int value)
{
    this->ui->lbKernelBlurSize->setNum(2*value+1);
}

void CalibColor::changeLabelClosingKernelSize(int value)
{
   this->ui->lbKernelClosingSize->setNum(2*value+1);
}

void CalibColor::on_btnFinish_clicked()
{
    this->calib->stop();
    this->calib->exit(0);
    this->hide();
    this->parent->show();
}


void CalibColor::on_btnStartCalib_clicked()
{
    if(this->ui->btnStartCalib->text()=="Start Calibrate")
    {
        this->ui->btnStartCalib->setText("Stop Calibrate");
        this->calib->start();
    }
    else
    {
        this->ui->btnStartCalib->setText("Start Calibrate");
        this->calib->stop();
    }
}


void CalibColor::on_btnSave_clicked()
{
    std::string path = this->ui->leSaveDir->text().toStdString();
    this->calib->save(path);
}


void CalibColor::on_pushButton_clicked()
{
    this->ui->leSaveDir->setText(QFileDialog::getExistingDirectory(this, "Choose directory", QDir::homePath()));
}
