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

Future ToDo: add transdec support
"""
cuprint = CUPrint("Prior GUI", ros=False)

class ClickableLabel(QLabel):

   def __init__(self, figure, task, clicked_func, rotation=0, widget_size=(480, 640)):
      super(ClickableLabel, self).__init__()
      self.clicked_func = clicked_func
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
      self.setMaximumSize(120, 120)
   
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
      self.clicked_func(self.task)

class Cusub_GUI(QWidget):
   def __init__(self, map_name, map_config, mission_config, simulation):
      self.map_config = map_config
      self.mission_config = mission_config
      self.map_name = map_name
      self.simulation = simulation

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
         label = ClickableLabel("figures/" + fig_name, task, self.clicked_event, rotation=rot)
         self.tasks[label] = qp

      font = QFont("Arial",16)

      self.active_task_label = QLabel("None Selected")
      self.active_task_label.setFont(font)
      self.active_task = None


      self.edit_prior_depth_label = QLabel("Z prior: ")
      self.edit_prior_depth_label.setFont(font)
      self.edit_prior_depth = QLineEdit()
      self.edit_prior_depth.setFont(font)
      self.edit_prior_depth.setValidator(QDoubleValidator())
      self.edit_prior_depth.setMaxLength(4)
      self.edit_prior_depth.textChanged.connect(self.prior_depth_edited)

      self.edit_sim_depth_label = QLabel("Z sim: ")
      self.edit_sim_depth_label.setFont(font)
      self.edit_sim_depth = QLineEdit()
      self.edit_sim_depth.setFont(font)
      self.edit_sim_depth.setValidator(QDoubleValidator())
      self.edit_sim_depth.setMaxLength(4)
      self.edit_sim_depth.textChanged.connect(self.sim_depth_edited)

      map_name = self.map_name.capitalize()
      if self.simulation:
         map_name += " Simulation Run"
      else:
         map_name += " Hardware Run"
      map_label = QLabel(map_name)
      map_label.setFont(QFont("Arial", 24))
      update_mission_config = QPushButton("Update mission config")
      update_mission_config.clicked.connect(self.save_mission_config)
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

      reload_button = QPushButton("Reload")
      reload_button.setFont(font)
      reload_button.clicked.connect(self.reload_mission_config)
      
      grid = QGridLayout()
      
      grid.addWidget(self.main_label,0,0)
      for t in self.tasks.keys():
         grid.addWidget(t, 0,0)
      grid.addWidget(check_box, 0,1, alignment=Qt.AlignCenter)
      grid.addWidget(grid_button, 0,1, alignment=Qt.AlignTop)
      grid.addWidget(reload_button, 0,1, alignment=Qt.AlignBottom)

      # Depth Adjustments
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

      x_pixels = self.map_config["map_dims"]["x_pixels"]
      y_pixels = self.map_config["map_dims"]["y_pixels"]
      self.setGeometry(0,0,x_pixels,y_pixels)
      self.main_label.installEventFilter(self)
      
      self.setWindowTitle("Cusub GUI")
      self.show()
      
      for t in self.tasks.keys():
         t.move(self.tasks[t])

   def reload_mission_config(self):
      os.chdir("../config/")
      self.mission_config = load_mission_config()
      os.chdir("../scripts/")
      cuprint("reloaded mission config")

   def sim_depth_edited(self, text):
      if self.active_task:
         task_name = self.active_task
         model = self.map_config["priors"][task_name]["sim_model"]
         try: # partial text fails str -> float conversion
            self.map_config["sim_truth_depths"][model] = float(text)
         except ValueError:
            pass

   def prior_depth_edited(self, text):
      if self.active_task:
         task_name = self.active_task
         try: # partial text fails str -> float conversion
            if "prior_depth" in self.map_config["priors"][task_name].keys():
               self.map_config["priors"][task_name]["prior_depth"] = float(text)
         except ValueError:
            pass
   
   def clicked_event(self, task_name):
      self.active_task = task_name
      self.active_task_label.setText(task_name)
      prior_z = 0.0
      if "prior_depth" in self.map_config["priors"][task_name].keys():
         prior_z = self.map_config["priors"][task_name]["prior_depth"]
      model = self.map_config["priors"][task_name]["sim_model"]
      sim_z = 0.0
      if model in self.map_config["sim_truth_depths"].keys():
         sim_z = self.map_config["sim_truth_depths"][model]
      else:
         cuprint("could not find model: " + model + " in map config file",warn=True)
      self.edit_prior_depth.setText(str(prior_z))
      self.edit_sim_depth.setText(str(sim_z))

   def save_mission_config(self):
      x_pixels = self.map_config["map_dims"]["x_pixels"]
      y_pixels = self.map_config["map_dims"]["y_pixels"]
      x_len = self.map_config["map_dims"]["x_len"]
      y_len = self.map_config["map_dims"]["y_len"]

      x_meters_per_pixel = x_len / x_pixels
      y_meters_per_pixel = y_len / y_pixels

      lev = [x for x in self.tasks.keys() if x.task == "leviathan"][0]
      qt_lev_x = self.tasks[lev].x() + self.map_config["map_dims"]["fig_qt_offset"][0]
      qt_lev_y = self.tasks[lev].y() + self.map_config["map_dims"]["fig_qt_offset"][1]

      vert_axis_down = self.map_config["map_dims"]["image_vertical_axis_down"]
      horiz_axis_right = self.map_config["map_dims"]["image_horizontal_axis_right"]

      for t in self.tasks.keys():
         task_name = t.task
         if task_name == "leviathan":
            continue

         qtx = self.tasks[t].x() + self.map_config["map_dims"]["fig_qt_offset"][0]
         qty = self.tasks[t].y() + self.map_config["map_dims"]["fig_qt_offset"][1]

         qtx_diff = qtx - qt_lev_x
         qty_diff = qty - qt_lev_y
         x_diff = qtx_diff * x_meters_per_pixel
         y_diff = qty_diff * y_meters_per_pixel

         dist = np.linalg.norm([x_diff, y_diff])

         if self.simulation: # Simulation coord frame never changes
            x_prior, y_prior = self.transform_model_coord(x_diff, y_diff, vert_axis_down, horiz_axis_right, 0, 0)
         else: # hardware frame changes, wherever we zero the IMU at
            y_diff = -y_diff
            qt_lev_rot = -lev.rotation + 90 # REAL
            theta = np.arctan2(y_diff, x_diff) - (qt_lev_rot * (np.pi/180))
            x_prior = np.cos(theta) * dist
            y_prior = np.sin(theta) * dist

         z_prior = self.map_config["priors"][task_name]["prior_depth"]
         
         x_prior = round(x_prior, 2)
         y_prior = round(y_prior, 2)
         z_prior = round(z_prior, 2)

         self.mission_config["tasks"][task_name]["prior"] = [x_prior, y_prior, z_prior]

      filename = "../config/" + "mission_config_generated.yaml"
      with open(filename, 'w') as f:
        yaml.dump(self.mission_config, f)
      cuprint("mission config saved as " + bcolors.HEADER + "mission_config_generated.yaml" + bcolors.ENDC)

      self.save_model_locs()

   def save_model_locs(self):
      x_pixels = self.map_config["map_dims"]["x_pixels"]
      y_pixels = self.map_config["map_dims"]["y_pixels"]
      x_len = self.map_config["map_dims"]["x_len"]
      y_len = self.map_config["map_dims"]["y_len"]
      
      x_meters_per_pixel = x_len / x_pixels
      y_meters_per_pixel = y_len / y_pixels

      center_x_world, center_y_world = self.map_config["map_dims"]["world_coords_of_top_left_corner_of_image"]

      vert_axis_down = self.map_config["map_dims"]["image_vertical_axis_down"]
      horiz_axis_right = self.map_config["map_dims"]["image_horizontal_axis_right"]

      model_locs = {}
      model_locs["mappings"] = {}
      for t in self.tasks.keys():
         qtx = self.tasks[t].x() + self.map_config["map_dims"]["fig_qt_offset"][0]
         qty = self.tasks[t].y() + self.map_config["map_dims"]["fig_qt_offset"][1]

         task_name = t.task
         model_name = self.map_config["priors"][task_name]["sim_model"]

         x = qtx * x_meters_per_pixel
         y = qty * y_meters_per_pixel
         z = self.map_config["sim_truth_depths"][model_name]
         rot = -t.rotation * (np.pi / 180) # qt pos angle is clockwise
         if task_name == "leviathan":
            rot += 180 * (np.pi/180) # flip leviathan 180 deg
         if task_name == "torpedos":
            rot += np.pi

         x,y = self.transform_model_coord(x, \
                                          y, \
                                          vert_axis_down, \
                                          horiz_axis_right, \
                                          center_x_world, \
                                          center_y_world)
         
         x = round(x, 2)
         y = round(y, 2)
         z = round(z, 2)
         rot = round(rot, 2)

         model_locs[model_name] = [x, y, z, rot]

         if task_name != "leviathan":
            mappings = {}
            mappings["model"] = model_name
            mappings["prior_z"] = self.map_config["priors"][task_name]["prior_depth"]
            model_locs["mappings"][task_name] = mappings

      filename = "../../../cusub_sim/stress_tester/config/model_locs.yaml"
      with open(filename, 'w') as f:
        yaml.dump(model_locs, f)

   # transforms qt coord to world coord in gazebo
   def transform_model_coord(self, horiz, vert, vert_axis_down, horiz_axis_right, center_x, center_y):
      x, y = None, None
      if "x" in vert_axis_down: # flip coords
         x = vert
         y = horiz
      else:
         x = horiz
         y = vert
      
      if "-" in vert_axis_down:
         x = -x
      if "-" in horiz_axis_right:
         y = -y
      
      x = x + center_x
      y = y + center_y

      return x, y

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
      new_pos = e.pos() - QPoint(54, 54)
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

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')
	
if __name__ == '__main__':
   folder = os.getcwd()
   if folder[-7:] != "scripts":
      cuprint("Please run configure_run.py in the scripts folder")

   # Load args
   parser = argparse.ArgumentParser(description='Place priors GUI')
   parser._action_groups.pop()
   required = parser.add_argument_group('required arguments')
   optional = parser.add_argument_group('optional arguments')
   required.add_argument("-s", "--simulation", type=str2bool, help="t/f, simulation run?", required=True)
   optional.add_argument("-m", "--map", default="dw", type=str, help='map label: a, b, c, d, f (finals), dw (divewell). Default: dw')
   args = parser.parse_args()

   if args.simulation:
      cuprint("Simulation Run")
   else:
      cuprint("Hardware Run")

   # Load configs
   map_name, map_config = load_map(args.map.lower())
   mission_config = load_mission_config()

   # Load GUI
   os.chdir("../scripts")
   app = QApplication(sys.argv)
   ex = Cusub_GUI(map_name, map_config, mission_config, args.simulation)
   sys.exit(app.exec_())