#include "interface.h"
#include "ui_interface.h"



interface::interface(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::interface)
{
    ui->setupUi(this);




}

interface::~interface()
{
    delete ui;
}

void interface::on_Start_camera_clicked()
{
   ui->textEdit->setText("Clicca sull'immagine quale bottone vuoi premere");


}
