********************
robosub_descriptions
********************

The Robosub Descriptions folder contains every object for the sim that is not the sub models. 

World and Model Folders
#######################

This includes the various scenarios we would want to simulate, and models for every object which the subs interact with.  

All the models require their own stl or dae file along with their respective xacro for the object. Since these models are not broken up into several different xacro files like the subs, all meshes, connectors, weights, hydrodynamic parameters, inertial models, and collision geometry must go into the one file. Basic world information, also go into the models folder and are referenced by the worlds package. 

Other Folders
#############

The launch, config, and src folders are all extra resources you can create for the sim. 

The src folder would hold python scripts for events, like if an object is dispensed from a machine. 

The launch folder holds the standard upload.launch and other testing launch files. 

And the config folder holds extra joint information and yaml files to accompany the src scripts.