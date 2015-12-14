# Analysis 

## Robosub 2015 Downfall
1. Classification of objects under poor conditions 
2. Fail to assign probablity to partially occluded object 
3. Non-uniform illumination and underwater caustics for bottom tasks 
4. Fail to make use of prior knowledge about the task
5. Difficult to distinguish object purely on color when needed 
6. Lack of multiple strategies for each task
7. Did not stress algorithms enough during pool test 

## Categories of tasks 
1. Lane marker
  - orange color. rectangle. 
  - obtain heading from lane marker
  - false positive when near other object 
  - use aspect ratio 
  - prone to breaking into two parts or holes due to bubbles and tether

2. Buoys
  - each buoy has tint of blue or green 
  - hard to distinguish based on color alone 
  - different hard required scanning at different height 
  - top part of buoy is washed up making it hard to obtain full sphere 

3. Poles
  - detecting green color underwater or yellow not reliable 
  - termination criteria 

4. Shooting 
  - locating center of board without any skewness 
  - multiple color combinations 

5. Front manipulation 
  - Combination of colors in one object
  - typically very small object 
  - precision 

6. Acoustic combo 
  - Need to identify correctly octagon 
  - Use vision cue to determine center of octagon 
  - staying very close to object affects vision detection 
  
## Prior information
  - estimated location 
  - size of actual object
  - what is around the object like other buoy 
  - orientation of object 
  - color of object
  - time to find object 

## Things that needed to be done at Robosub 
  -  automate checking of system 
  -  retrieve and analyze bags quickly after each practice round 
  -  sync code with vehicle efficiently
  -  integrated tests for code 
  -  change and save parameters quickly
  -  backup plans for every state in taskrunner 
  -  list of things to be tested during practice run for a particular task 
  -  systematic ways to collect and annotate data 
