# TODO

## 1.Research 
- red channel prior to dehaze image 
- SLAT algorithm to segment degraded color image 
- explore different features i.e HOG, LBP, FAST, KAZE, 
- saliency detection 
- ensemble method i.e boosting 
- bayesian filtering to make use of prior information 
- distance estimation from monocular image 
- combining global and local features
- combining discriminative, generative and temporal model 
- particle filtering 
- online and offline learning 
- maximum margin correlation filter MOSSE
- deformable part model and usage of bag of words
- using rnn and cnn to classify image 
- using svm 
- color constancy 

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

