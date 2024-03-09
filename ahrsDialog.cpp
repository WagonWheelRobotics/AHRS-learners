#include "ahrsDialog.h"
#include "ui_ahrsDialog.h"

#include "ubxDecoder.h"
#include "imuPacket.h"
#include "qcpPlotView.h"

#include <QSerialPort>
#include <QDebug>

ahrsDialog::ahrsDialog(QSerialPort *port, QList<qcpPlotView *> plots, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ahrsDialog)
{
    ui->setupUi(this);
    _plots = plots;
    _port = port;
    _decoder = new ubxDecoder('W','2',65000);
    connect(_port, SIGNAL(readyRead()), this, SLOT(ready()));
}

ahrsDialog::~ahrsDialog()
{
    delete ui;
    _port->close();
    _port->deleteLater();
    delete _decoder;

    qDebug()<<"ahrsDialog::~ahrsDialog";
}



void ahrsDialog::ready()
{
    auto bytes=_port->readAll();
    for(const auto &i:bytes)
    {
        auto r=_decoder->receive(i);
        if(r==1)
        {
            auto raw = _decoder->raw();
            if(raw[2]==CLASS_LOG && raw[3]==ID_ICM42688)
            {
                const imuPacket_t *p = (const imuPacket_t*) raw;
                ui->leTime->setText(QString("%1").arg(p->time));
                ui->leTemp->setText(QString("%1").arg(p->temp));
                ui->leGx->setText(QString("%1").arg(p->imu[0]));
                ui->leGy->setText(QString("%1").arg(p->imu[1]));
                ui->leGz->setText(QString("%1").arg(p->imu[2]));
                ui->leAx->setText(QString("%1").arg(p->imu[3]));
                ui->leAy->setText(QString("%1").arg(p->imu[4]));
                ui->leAz->setText(QString("%1").arg(p->imu[5]));

                QVector<double> data;
                data.append(p->time*1e-5);
                data.append(p->imu[0]);
                data.append(p->imu[1]);
                data.append(p->imu[2]);
                _plots[0]->addData(data);

                data[1] = p->imu[3];
                data[2] = p->imu[4];
                data[3] = p->imu[5];
                _plots[1]->addData(data);
            }
        }
        else if(r<0)
        {
            //checksum error
            qWarning()<<"Checksum error";
        }
    }
}

QList<qcpPlotView *> &ahrsDialog::plots()
{
    return _plots;
}
