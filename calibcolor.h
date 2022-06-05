#ifndef CALIBCOLOR_H
#define CALIBCOLOR_H

#include <iostream>
#include <string>

#include <QDialog>

#include "calibration.h"
#include "convert.h"
#include "controllib.h"


namespace Ui {
class CalibColor;
}

class CalibColor : public QDialog
{
    Q_OBJECT

public:
    explicit CalibColor(QWidget *parent = nullptr);
    ~CalibColor();

public slots:
    void streamShow(QImage filter, QImage mask);
    void refFrameShow(QImage refFrame);

    void changeLabelBlurKernelSize(int value);
    void changeLabelClosingKernelSize(int value);

private slots:
    void on_btnFinish_clicked();

    void on_btnStartCalib_clicked();

    void on_btnSave_clicked();

    void on_pushButton_clicked();

private:
    Ui::CalibColor *ui;
    QWidget *parent;
    ColorCalibration *calib;
};

#endif // CALIBCOLOR_H
