# Detection Tree Package

## Block Diagrams
Can be found in google drive under Engineering/Software/Block Diagrams/Detection Tree/

## Data Structures and Messages
The two data structures used by this package are dvectors and dobjects. The 'd' stands for _detection_. 

A d-vector is a detection vector; a vector of the bearing from the sub's current position to the detection. Specifically, the bearing is a global azimuth and elevation from the sub's global position.

A d-object is simply an object that we've recognized. When we receive a darknet hit and create the associating d-vector we assign that d-vector to a d-object. So, a d-object is a grouping of d-vectors. Why not just group by class-id? If we only had one of each class we could do this... alas, we have 2 start gate poles. Each pole is a separate instance of class "start_gate_pole," so we need to group our detections based on objects not classes.