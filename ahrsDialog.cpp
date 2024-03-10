#include "ahrsDialog.h"
#include "ui_ahrsDialog.h"

#include "ubxDecoder.h"
#include "imuPacket.h"
#include "qcpPlotView.h"
#include "imuCore.h"
#include "ahrsEulerCF.h"

#include <QSerialPort>
#include <QDebug>

ahrsDialog::ahrsDialog(QSerialPort *port, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ahrsDialog)
{
    ui->setupUi(this);
    _port = port;

    _imu = new imuCore;
    _ahrs = new ahrsEulerCF(1.0f/200.0f);

    _decoder = new ubxDecoder('W','2',65000);
    connect(_port, SIGNAL(readyRead()), this, SLOT(ready()));
}

ahrsDialog::~ahrsDialog()
{
    delete ui;
    _port->close();
    _port->deleteLater();
    delete _decoder;

    delete _imu;

    qDebug()<<"ahrsDialog::~ahrsDialog";
}

void ahrsDialog::setPlots(QList<qcpPlotView *> &plots)
{
    _plots = plots;
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

                float corImu[6];
                _imu->get(p->imu, corImu);

                std::vector<float> output;
                _ahrs->update(corImu,output);

                QVector<double> data;
                data.append(p->time*1e-5);

                data.append(corImu[0]);
                data.append(corImu[1]);
                data.append(corImu[2]);
                _plots[0]->addData(data);

                data[1] = corImu[3];
                data[2] = corImu[4];
                data[3] = corImu[5];
                _plots[1]->addData(data);

                data.resize(output.size()+1);   //+1 for timestamp
                data[0] = p->time*1e-5;
                for(size_t i=0;i<output.size();i++)
                {
                    data[1+i] = output[i];
                }
                _plots[2]->addData(data);

                float e[3];
                _ahrs->getEuler(e);
                emit updatePose(e);
#if 0
                data.append(p->imu[0]);
                data.append(p->imu[1]);
                data.append(p->imu[2]);
                _plots[0]->addData(data);

                data[1] = p->imu[3];
                data[2] = p->imu[4];
                data[3] = p->imu[5];
                _plots[1]->addData(data);
#endif
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

QStringList ahrsDialog::output()
{
    QStringList header;
    header.append("time");
    for(const auto &i:_ahrs->output())
    {
        header.append(QString::fromStdString(i));
    }
    return header;
}


