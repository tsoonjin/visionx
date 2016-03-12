## Developing an expertise in Object Tracking for Autonomous Systems
Autonomous systems are increasingly more prominent with increase in computing power and 
more novel algorithms that enables the robot to make decision intelligently. Object tracking
remains one of the most important task for an autonomous system such as scanning underwater 
gas pipeline, surveillance, self-driving car etc. A good tracking algorithm exploits domain
specific knowledge to improve efficiency;therefore, this research mainly focuses on underwater
application of object tracking.

### Long-term goals
- [ ] fully-integrated multiple object tracking algorithm with ROS(Robotics Operating System)
- [ ] robust single object tracking algorithm under perturbed conditions

### Short-term goals
- [ ] simple object recognition under various conditions
 - [ ] varying illumination conditions
 - [ ] occlusion 
 - [ ] shadow 
- [ ] multiple simple object recognitions under various conditions
- [ ] integrate vision processing module with ROS (Robotics Operating System) 


### Plans
1. Enroll in online courses 
 - Artificial Intelligence in Robotics, Sebastian Thrun, Udacity
 - University of Central Florida, Computer Vision 2012, Mubarak Shah 

2. Take CS4243 Computer Vision and Pattern Recognition 

### Different approaches
1. Single Object Tracking
  - matching
    - region -> uses object template(global information) and similarity measure i.e mean shift  
    - feature -> uses local information involves feature extraction and feature matching 
    - deformable template -> uses contour that is highly elastic for tracking but sensitive to initialization 
    - model -> uses geometric model 
  - filtering
    - kalman 
    - particle 
  - class -> regard as classification btw background and foreground
  - fusion
    - multi-feature
    - multi-algorithm
    - multi-model 
2. Multiple Object Classification 
3. Saliency Region Detection 
4. Underwater Image Recovery 
5. Justifying background subtraction 
6. Using power of context and semantics
7. Hybrid between global and local. Offline and online 

## Novel ideas 
- combining multiple algorithms or features generally yield better result 
- using prior information to advantage through Bayesian Filtering 
- exploit temporal cues such as optical flow or other motion detector 
- using probabilistic model instead of binary values. focus on soft detection 
- ensemble methods and cascade style detection is very efficient 
- add usage of features and templates as part of algorithm

## Robosub deliverable
- saliency region with opponent color boosting
- saliency with local entropy 
- color-rabbit 
- image recovery for illumination, haze and shadow
- using simple features for particle based tracking 
- using temporal cue such as optical flow and motion estimation 
- deep learning for feature extraction
- fast r-cnn for object detection
