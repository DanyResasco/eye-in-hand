#ifndef INTERFACE_H
#define INTERFACE_H

#include <QMainWindow>
#include <QtGui/QMovie>
#include <QtWidgets/QWidget>
#include "opencv2/highgui/highgui.hpp"

namespace Ui
{
class interface;
}

class interface : public QMainWindow
{
    Q_OBJECT

public:
    explicit interface(QWidget *parent = 0);
    ~interface();

private slots:
    void on_Start_camera_clicked();

private:
    Ui::interface *ui;


};

#endif // INTERFACE_H
