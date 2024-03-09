#ifndef AHRSDIALOG_H
#define AHRSDIALOG_H

#include <QDialog>

namespace Ui {
class ahrsDialog;
}

class QSerialPort;
class ubxDecoder;
class qcpPlotView;

class ahrsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ahrsDialog(QSerialPort *port, QList<qcpPlotView *> plots, QWidget *parent = nullptr);
    ~ahrsDialog();

    QList<qcpPlotView *> &plots();

private slots:
    void ready(void);

private:
    Ui::ahrsDialog *ui;

    QSerialPort *_port;
    QList<qcpPlotView *> _plots;

    ubxDecoder *_decoder;
};

#endif // AHRSDIALOG_H
