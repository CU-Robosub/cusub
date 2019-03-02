**************
gazebo_drivers
**************
The gazebo drivers package stores all the scripts to fake the sensor data in gazebo. 

**scripts folder**: Every sensor needs its own python script to work properly in the sim and if you are using the yolo faker to avoid gpu dependent tasks, you also need yaml camera files. These yamls are stored in the config folder and it also contains the actual calibration properties from the real world. These are used so that gazebo can properly distort the simulated image. 

**launch folder**: Each launch file starts the python scripts of the appropriate sensor that the sub uses. These are dependent on sub and if new sensors are added, make sure to create the new python scripts along with adding them to the launch, so that gazebo can properly incorporate all the data the sub would experience. 
