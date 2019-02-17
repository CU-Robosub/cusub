#ifndef QT_DIALOG_H
#define QT_DIALOG_H

#include <QDialog>

namespace Ui {
class qt_dialog;
}

class qt_dialog : public QDialog
{
  Q_OBJECT

public:
  explicit qt_dialog(QWidget *parent = 0);
  ~qt_dialog();

private:
  Ui::qt_dialog *ui;
};

#endif // QT_DIALOG_H
