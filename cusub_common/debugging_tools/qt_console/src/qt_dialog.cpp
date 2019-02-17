#include "qt_dialog.h"
#include "ui_qt_dialog.h"

qt_dialog::qt_dialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::qt_dialog)
{
  ui->setupUi(this);
}

qt_dialog::~qt_dialog()
{
  delete ui;
}
