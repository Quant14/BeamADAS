# BeamADAS
This is a project, aimed at developing a realistic ADAS system using BeamNG.tech as a platform for accurate vehicle simulation.

## Goals of the ADAS system
1. Detect road curvature and slow down if the vehicle speed is too high.
2. Smoothly reduce the closing speed to an object in the current road lane to 0 and continue driving, following from a safe distance for the current speed.
3. Indicate to the driver in case there is a car in either of the side mirrors' blind spots.
4. If the vehicle is moving at a speed <12 kph (e.g. parking) and an ultrasonic sensor detects too low distance, brakes are applied to avoid contact with any objects.
5. (Future) If the vehicle is exiting its road lane without using the relative indicator, slightly pull the steering back towards the current road lane.
6. (Future) Track vehicle and other objects' trajectories for more accurate LiDAR system.
7. (Future) Implement automatic system diagnostics in case of module failure.

## Relation between host and processor
The host uses BeamNGpy to get live sensor data from BeamNG.tech and send it to the processor (separate microcomputer, connected via USB). The processor analyzes the sensor data and sends back to the host if action needs to be taken and what (set pedal inputs to x%). The host can then apply those changes to the vehicle inputs inside the simulation (a small BeamNG mod package is required). 

## System diagram
<img src="https://github.com/Quant14/BeamADAS/blob/main/media/beamadas_diagram.png?raw=true">

## Progress log
#### 28th May 2024
The project's main goals have been achieved. Its functionalities have a lot of room for improvement but it is a solid proof of concept that a realistic multi-process multi-device ADAS system can be implemented to work with BeamNG.tech. 
#### 1st Mar 2024
The progress log of the project is NOT up to date. The project is in active development with big progress steps pushing it towards completion. 
#### 1-3rd Nov 2023
Successful LiDAR data transfer at reasonable speed. LiDAR resolution reduced for performance reasons.
Image data transfer testing. Huge serial port bottleneck. Solution required immediately for faster image transfer.
Testing planning.
#### 23-27th Oct 2023
Completed official requirements sheet.
Set up successful communication between host and processor. 
Discovered issues related to Raspberry Pi4 processing speed. 
Created naive lane weight system (needs testing).
#### 16-20th Oct 2023
Improved road curvature detection accuracy.
Started work on a way to calculate lane weights and use the curvature radius value only of the lane that is accurately calculated.
Created a system diagram of the project.
Started work on an official requirements sheet for the project.
#### 9-13th Oct 2023
Testing of BeamNG.tech v0.30.5 sensor streaming and performance benchmarks.
Switched to pausing concept (pausing the simulation while waiting for sensor data to be received).
Big advancement in controlling the vehicle by filtering out user pedal inputs when needed (needs testing).
Serial communication with Raspberry Pi research.
Generated new camera testing images and started work on road curvature detection consistency and accuracy.
#### 2-6th Oct 2023
Adjustment of project requirements based on new information about the issues found the previous week.
Raspberry Pi initial setup.
Research into Lua and preventing interference between the driver's inputs and the ADAS' inputs.
Start upgrade from BeamNG.tech v0.29.1 to v0.30.5
#### 25-29th Sep 2023
Further specification of project requirements.
Identified issues related to sensors latency and LiDAR data analysis. 
Communication with the BeamNG.tech team related to mentioned issues.
#### 18-22nd Sep 2023
Ultrasonic sensor setup and visualization: https://clipchamp.com/watch/7sZU0Cyv0Wi

Improved sensor settings.
Added AI control and testing.
Added emergency braking test.
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
