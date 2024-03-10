#ifndef AHRSDIALOG_H
#define AHRSDIALOG_H

#include <QDialog>

namespace Ui {
class ahrsDialog;
}

class QSerialPort;
class ubxDecoder;
class qcpPlotView;

class imuCore;
class ahrsCore;

class ahrsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ahrsDialog(QSerialPort *port, QWidget *parent = nullptr);
    ~ahrsDialog();

    void setPlots(QList<qcpPlotView *> &plots);
    QList<qcpPlotView *> &plots();
    QStringList output();

private slots:
    void ready(void);

signals:
    void updatePose(float *euler);

private:
    Ui::ahrsDialog *ui;

    QSerialPort *_port;
    QList<qcpPlotView *> _plots;

    ubxDecoder *_decoder;

    imuCore *_imu;
    ahrsCore *_ahrs;
};

#endif // AHRSDIALOG_H
