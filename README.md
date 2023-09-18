# BeamADAS
This is a project, aimed at developing a realistic ADAS system using BeamNG.tech as a platform for accurate vehicle simulation.

## Goals of the ADAS system
1. Detect road curvature and apply brakes if the vehicle speed is too high.
2. If the vehicle is exiting its road lane without using the relative indicator, slightly pull the steering back towards the current road lane.
3. Smoothly reduce the closing speed to an object in the current road lane to 0 and continue driving, following from a safe distance for the current speed.
4. If the vehicle controlled by BeamADAS can not slow down quickly enough, then try to steer to the right side of the road if enough space is available. (?)
5. If the vehicle controlled by BeamADAS is driving alongside another vehicle but the distance detected by the side ultrasonic sensor is too low, pull the steering away from that side (as long as enough space is available on the other side and said other side is not the opposite lane of traffic) and reduce speed so the two vehicles are not side by side.
6. If the vehicle is moving at a speed <15 kph (e.g. parking) and an ultrasonic sensor detects too low distance, brakes are applied to avoid contact with any objects.
7. If an object is closing in too quickly from behind and it is safe to do so, apply throttle to avoid collision from behind. (?)

## Relation between host and processor
The host uses BeamNGpy to get live sensor data from BeamNG.tech and send it to the processor (separate microcomputer, connected to the host via USB). The processor calculates the adjustments that need to be made to the inputs of the vehicle and sends them back to the host. The host can then apply those changes to the vehicle inputs inside the simulation. 

## Progress log
#### 11-15th Sep 2023
Successful semi-working road curvature detection.
LiDAR data output analysis.
#### 4-8th Sep 2023
Adjusted camera sensor FOV and adjusted road curvature algorithm for new road position.
Performed successful testing and adjustment of initial stages of road curvature detection algorithm.
Simple camera setup inside BeamNG.tech, generated test images, continued work on road curvature detection.
#### 30th Aug - 1st Sep 2023
Camera specification research, started implementing road curvature detection based on:

https://github.com/OanaGaskey/Advanced-Lane-Detection

LiDAR research, simple LiDAR setup in different situations. Screenshots made inside BeamNG.tech.

<img src="https://github.com/Quant14/BeamADAS/blob/main/media/highway_hood.png?raw=true" width=60%>
<img src="https://github.com/Quant14/BeamADAS/blob/main/media/highway_outside.png?raw=true" width=60%>
<img src="https://github.com/Quant14/BeamADAS/blob/main/media/countryside_hood.png?raw=true" width=60%>
<img src="https://github.com/Quant14/BeamADAS/blob/main/media/countryside_outside.png?raw=true" width=60%>
<img src="https://github.com/Quant14/BeamADAS/blob/main/media/town_hood.png?raw=true" width=60%>
<img src="https://github.com/Quant14/BeamADAS/blob/main/media/town_outside.png?raw=true" width=60%>

Project organization and requirements, installation of BeamNG.tech and initial setup.
