

Check out the [latest documentation](https://cusub.readthedocs.io/en/latest/). (Work in Progress)


Robosub Model
--------------
![Robot Model](./cusub_documentation/source/images/robosub_system_interconnect_model.jpg)




Project layout
--------------
```
├── cusub_bringup/
│     ├── config/
│     ├── launch/
│     └── scripts/
│
├── cusub_common/
│     ├── ar_tracking/                  
│     ├── cusub_common/
│     ├── cusub_common_bringup/         
│     ├── debugging_tools/
│     ├── depreciated/
│     ├── drivers/                     
│     ├── motor_controllers/          
│     └── waypoint_navigator/
│
├── cusub_cortex/
│     ├── cusub_cortex/
│     ├── cusub_cortex_bringup/
│     ├── mapper/
│     └── state_machine/
│
├── cusub_documentation/
│     └── source/
│
├── cusub_perception/
│     ├── cusub_perception/
│     ├── cusub_perception_bringup/
│     ├── darknet_config/
│     ├── darknet_multiplexer/
│     ├── darknet_ros/
│     ├── detection_tree/
│     ├── localizer/
│     └── tracking/
│
└── cusub_sim/
      ├── cusub_sim/
      ├── cusub_sim_bringup/
      ├── leviathan_description/
      ├── robosub_descriptions/
      ├── stress_tester/
      ├── triton_description/
      └── uuv_simulator_test/
```
--------------
