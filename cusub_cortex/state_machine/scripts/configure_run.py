#!/usr/bin/env python
from __future__ import division
import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import numpy as np
import time

"""
sudo apt-get install qtcreator pyqt5-dev-tools

Steps:
Current problem: we can click the correct widget, but for some reason they still both move & orientation seems fucked up

- add grids onto the image
- convert + output a config file w/ priors
- add ability to set the z value through a box for prior & sim_gt
- add transdec sim + transdec real maps for each quadrant! (config files for each)
- connect to pipeline w/ symlinks etc
"""

class ClickableLabel(QLabel):

   def __init__(self, figure, task, widget_size=(480, 640)):
      super(ClickableLabel, self).__init__()
      self.being_dragged = False
      self.pixmap = QPixmap(figure)
      self.pixmap = self.pixmap.scaledToWidth(70)
      diag = (self.pixmap.width()**2 + self.pixmap.height()**2)**0.5
      self.setMinimumSize(diag, diag)
      self.setAlignment(Qt.AlignCenter)
      self.setPixmap(self.pixmap)
      self.rotation = 0
      self.widget_size = widget_size
      self.task = task
      self.setMaximumSize(80,80)
      self.modified = False
      
   def mouseMoveEvent(self, e):
      self.being_dragged = True
      mimeData = QMimeData()
      drag = QDrag(self)
      drag.setMimeData(mimeData)
      dropAction = drag.exec_(Qt.MoveAction)
      
      
   def check_dragged(self):
      return self.being_dragged

   def clear_being_dragged(self):
      self.being_dragged = False

   def check_modified(self):
      return self.modified

   def clear_modified(self):
      self.modified = False

   def mousePressEvent(self, event):
      modifiers = QApplication.keyboardModifiers()

      if modifiers == Qt.ShiftModifier:
         if event.buttons() == Qt.LeftButton:
            self.rotation += 15
         else:
            self.rotation -= 15
         transform = QTransform()
         transform.rotate(self.rotation)
         pixmap = self.pixmap.transformed(transform, Qt.SmoothTransformation)
         
         self.setPixmap(pixmap)

class Cusub_GUI(QWidget):
   def __init__(self):
      self.dragged = False
      self.tasks = {}
      super(Cusub_GUI, self).__init__()

      self.setAcceptDrops(True)

      im = QPixmap("divewell.png")
      label = QLabel()
      label.setPixmap(im)

      # im2 = QPixmap("figures/purple.png")
      # label2 = QLabel()
      # label2.setPixmap(im2)

      label2 = ClickableLabel("figures/red.png", "red")
      label3 = ClickableLabel("figures/purple.png", "purple")
      label4 = ClickableLabel("figures/orange.png", "orange")
      
      grid = QGridLayout()
      grid.addWidget(label,0,0)
      grid.addWidget(label2, 0,0)
      grid.addWidget(label3, 0,0)
      grid.addWidget(label4, 0,0)
      self.setLayout(grid)

      self.setGeometry(0,0,480,640)
      label.installEventFilter(self)
      
      self.setWindowTitle("Cusub GUI")
      self.show()

      # Works!
      self.tasks[label2] = QPoint(80, 238)
      self.tasks[label3] = QPoint(100,100)
      self.tasks[label4] = QPoint(200,200)
      for t in self.tasks.keys():
         t.move(self.tasks[t])
      
   def eventFilter(self, obj, event):
      if self.dragged:
         self.dragged = False
      elif event.type() == 12: # Paint event, move object back
         for t in self.tasks.keys():
            t.move(self.tasks[t])             
      return False
      
   def dragEnterEvent(self, e):
      e.accept()

   def dropEvent(self, e):
      self.dragged = True
      new_pos = e.pos() - QPoint(50, 50)
      for task in self.tasks.keys():
         if task.check_dragged():
            task.clear_being_dragged()
            self.tasks[task] = new_pos
            task.move(new_pos)
      e.setDropAction(Qt.MoveAction)
      e.accept()
      
	
if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = Cusub_GUI()
   sys.exit(app.exec_())