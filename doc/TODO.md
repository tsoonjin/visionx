# TODO

## 1. Pooltest 
  - gather varying amount of bag data for first session under different illumination and scenarios 
  - start forming backup plan for each state early 
  - test motion early
  
## 2.Taskrunners
- arguments
  - without mission planner
  - static detection 
  - with mission planner (test mode)
- use communicator module to subscribe and publish topic 
- use logger that wraps around ros logger 
- record time taken for each state 
- record universal time taken when node exited

## 3.Detectors
- handle only opencv image format 
- define name of detector 
- implement detect method that return output object 
- update vision parameters from dynamic reconfigure 

## 4.DataType 
### Output
- centroid of target 
- isDetected  
- dy, dx
- ratio between area of object and screen 
- angle from current heading to long side of object 


## 5.Utils

### Visualizer
- default black bg
  - overlay centroid of object, center of screen, dy and dx value, ratio of area, isDetected, outermost contour 

### Preprocess 
### Threshold 
### Feature
### Enhancement
### Logger
### Stats 

## 6.Tool
- vision_gui
- trainer
- tuner 

## 7.Learning 

## 8.ROS Helper 
- BaseComm

## 9. Pipeline 

