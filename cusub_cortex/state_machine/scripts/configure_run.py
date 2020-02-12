#!/usr/bin/env python
from __future__ import division
import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import numpy as np
import time
import argparse
from cusub_print.cuprint import CUPrint, bcolors
import os
import yaml

"""
sudo apt-get install qtcreator pyqt5-dev-tools

Shift or control to turn an object, control gives more resolution

- add grids onto the image (probably make divewell_grided.png and swap the pixmap out)
- convert + output a config file w/ priors
- add ability to set the z value through a box for prior & sim_gt
- add transdec sim + transdec real maps for each quadrant! (config files for each)
- connect to pipeline w/ symlinks etc
"""
cuprint = CUPrint("Prior GUI", ros=False)

class ClickableLabel(QLabel):

   def __init__(self, figure, task, rotation=0, widget_size=(480, 640)):
      super(ClickableLabel, self).__init__()
      self.being_dragged = False
      self.rotation = rotation
      self.pixmap = QPixmap(figure)
      self.pixmap = self.pixmap.scaledToWidth(70)
      diag = (self.pixmap.width()**2 + self.pixmap.height()**2)**0.5
      self.setMinimumSize(diag, diag)
      self.setAlignment(Qt.AlignCenter)

      self.show_image()
      
      self.widget_size = widget_size
      self.task = task
      self.setMaximumSize(100,100)
   
   def show_name(self):
      font = QFont("Arial",18)
      self.setFont(font)
      self.setText(self.task)
      
   
   def show_image(self):
      transform = QTransform()
      transform.rotate(self.rotation)
      pixmap = self.pixmap.transformed(transform, Qt.SmoothTransformation)   
      self.setPixmap(pixmap)
      
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

   def mousePressEvent(self, event):
      modifiers = QApplication.keyboardModifiers()

      if modifiers == Qt.ShiftModifier:
         if event.buttons() == Qt.LeftButton:
            self.rotation += 15
         else:
            self.rotation -= 15
         self.show_image()
      elif modifiers == Qt.ControlModifier:
         if event.buttons() == Qt.LeftButton:
            self.rotation += 5
         else:
            self.rotation -= 5
         self.show_image()

class Cusub_GUI(QWidget):
   def __init__(self, map_name, map_config, mission_config):
      self.map_config = map_config
      self.mission_config = mission_config
      self.map_name = map_name

      self.dragged = False
      self.tasks = {}
      self.show_images = True
      super(Cusub_GUI, self).__init__()

      self.setAcceptDrops(True)

      self.main_label = QLabel()
      im = QPixmap("figures/" + self.map_name + ".png")
      self.main_label.setPixmap(im)
      self.gridded = False

      for task in self.map_config["priors"].keys():
         cuprint("placing " +bcolors.OKBLUE + task + bcolors.ENDC)
         fig_name = task + ".png"
         location = self.map_config["priors"][task]["qtlocation"]
         qp = QPoint(location[0], location[1])
         rot = location[2]
         label = ClickableLabel("figures/" + fig_name, task, rotation=rot)
         self.tasks[label] = qp

      font = QFont("Arial",16)

      self.active_task_label = QLabel("None Selected")
      self.active_task_label.setFont(font)


      self.edit_prior_depth_label = QLabel("Z prior: ")
      self.edit_prior_depth_label.setFont(font)
      self.edit_prior_depth = QLineEdit()
      self.edit_prior_depth.setFont(font)
      self.edit_prior_depth.setValidator(QDoubleValidator())
      self.edit_prior_depth.setMaxLength(4)
      # self.edit_prior_depth.setAlignment(Qt.AlignTop)

      self.edit_sim_depth_label = QLabel("Z sim: ")
      self.edit_sim_depth_label.setFont(font)
      self.edit_sim_depth = QLineEdit()
      self.edit_sim_depth.setFont(font)
      self.edit_sim_depth.setValidator(QDoubleValidator())
      self.edit_sim_depth.setMaxLength(4)
      # self.edit_sim_depth.setAlignment(Qt.AlignTop)

      map_label = QLabel("Divewell")
      map_label.setFont(QFont("Arial", 24))
      update_mission_config = QPushButton("Update mission config")
      update_mission_config.setFont(font)
      update_map_config = QPushButton("Update map config")
      update_map_config.clicked.connect(self.save_map_config)
      update_map_config.setFont(font)
      check_box = QPushButton("Toggle names")
      check_box.setFont(font)
      check_box.clicked.connect(self.show_names)

      grid_button = QPushButton("Show Grid")
      grid_button.setFont(font)
      grid_button.clicked.connect(self.show_grid)
      
      grid = QGridLayout()
      
      grid.addWidget(self.main_label,0,0)
      for t in self.tasks.keys():
         grid.addWidget(t, 0,0)
      # grid.addWidget(label3, 0,0)
      # grid.addWidget(label4, 0,0)
      grid.addWidget(check_box, 0,1, alignment=Qt.AlignCenter)
      grid.addWidget(grid_button, 0,1, alignment=Qt.AlignTop)

      # Depth
      # grid.addWidget(flo, 0, 2)
      grid.setColumnMinimumWidth(2, 100)
      grid.setColumnMinimumWidth(3, 200)
      grid.addWidget(self.active_task_label, 0,3)
      grid.addWidget(self.edit_prior_depth_label, 0,2, alignment=Qt.AlignTop)
      grid.addWidget(self.edit_sim_depth_label, 0,2, alignment=Qt.AlignBottom)
      grid.addWidget(self.edit_prior_depth, 0,3, alignment=Qt.AlignTop)
      grid.addWidget(self.edit_sim_depth, 0,3, alignment=Qt.AlignBottom)

      grid.addWidget(map_label,1,0, alignment=Qt.AlignCenter)
      grid.addWidget(update_mission_config, 2, 0)
      grid.addWidget(update_map_config, 3, 0)
      self.setLayout(grid)

      self.setGeometry(0,0,480,640)
      self.main_label.installEventFilter(self)
      
      self.setWindowTitle("Cusub GUI")
      self.show()
      
      for t in self.tasks.keys():
         t.move(self.tasks[t])

   def save_map_config(self):
      for task in self.map_config["priors"].keys():
         t = [x for x in self.tasks.keys() if x.task == task]
         if not t:
            cusub_print("couldn't locate " + task + " in self.tasks - there's a bug somewhere", warn=True)
         t = t[0]
         x = self.tasks[t].x()
         y = self.tasks[t].y()
         yaw = t.rotation
         self.map_config["priors"][task]["qtlocation"] = [x,y,yaw]
      
      # print(os.getcwd())
      filename = "../config/map_configs/" + self.map_name + ".yaml"
      with open(filename, 'w') as f:
        yaml.dump(self.map_config, f)
      cuprint("map configuration saved")

   def show_names(self):
      self.show_images ^= 1
      if self.show_images:
         for t in self.tasks.keys():
            t.show_image()
      else:
         for t in self.tasks.keys():
            t.show_name()

   def show_grid(self):
      self.gridded ^= 1
      if self.gridded:
         im = QPixmap("figures/" + self.map_name + "_gridded.png")
      else:
         im = QPixmap("figures/" + self.map_name + ".png")
      self.main_label.setPixmap(im)
      
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

def load_mission_config():
   mission_config = None
   with open("mission_config.yaml") as f:
      mission_config = yaml.load(f, Loader=yaml.FullLoader)
   return mission_config

def load_map(map_arg):
   os.chdir("../config/")
   map_config = None
   map_name = None
   if map_arg == "a":
      cuprint("loading " + bcolors.HEADER + "A" + bcolors.ENDC + " config")
   elif map_arg == "b":
      cuprint("loading " + bcolors.HEADER + "B" + bcolors.ENDC + " config")
   elif map_arg == "c":
      cuprint("loading " + bcolors.HEADER + "C" + bcolors.ENDC + " config")
   elif map_arg == "d":
      cuprint("loading " + bcolors.HEADER + "D" + bcolors.ENDC + " config")
   elif map_arg == "f":
      cuprint("loading " + bcolors.HEADER + "Finals" + bcolors.ENDC + " config")
   elif map_arg == "dw":
      cuprint("loading " + bcolors.HEADER + "Divewell" + bcolors.ENDC + " config")
      map_name = "divewell"
      with open("map_configs/divewell.yaml") as f:
         map_config = yaml.load(f, Loader=yaml.FullLoader)
   else:
      raise(Exception("Unrecognized map option: " + str(map_arg)))
   return map_name, map_config
	
if __name__ == '__main__':
   folder = os.getcwd()
   if folder[-7:] != "scripts":
      cuprint("Please run configure_run.py in the scripts folder")

   # Load args
   parser = argparse.ArgumentParser(description='Place priors GUI')
   parser.add_argument("-m", "--map", default="dw", type=str, help='map label: a, b, c, d, f (finals), dw (divewell). Default: dw')
   args = parser.parse_args()

   # Load configs
   map_name, map_config = load_map(args.map.lower())
   mission_config = load_mission_config()

   # Load GUI
   os.chdir("../scripts")
   app = QApplication(sys.argv)
   ex = Cusub_GUI(map_name, map_config, mission_config)
   sys.exit(app.exec_())