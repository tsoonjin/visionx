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
  -  extract useful information from collected data to give insight about collected data
  -  automatic caliberation of camera 

## Ideas
  - track other landmarks in the pool
  - underwater simulation to create perturbation and turbid condition of water in Robosub 
  
## ROS Nice Tools 
  - [using rqt_bag](http://wiki.ros.org/rqt_bag)
  - [using topic_tools](http://wiki.ros.org/topic_tools)
    - mux 
    - transform
    - drop 
  - [using visp](http://wiki.ros.org/visp) 
  - [using rosh](http://wiki.ros.org/rosh)
  - [using ecto](http://plasmodic.github.io/ecto/)
  - [using bag_tools](http://wiki.ros.org/bag_tools)
  - [writing plugin for rqt](http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin)
  - [using rostop](http://wiki.ros.org/rqt_top)
  - [using ParamEdit](http://wiki.ros.org/rosgui_paramedit)
  
## Useful libraries
  - [Caffe Deep Learning Framework](http://caffe.berkeleyvision.org/)
  - [Scikit-learn](http://scikit-learn.org/stable/)
  - [NoLearn: Wrapper for Caffe to work with Scikit](https://pythonhosted.org/nolearn/)
  - [OverFeat: CNN image classifier](https://github.com/sermanet/OverFeat)
  - [PyVision: MOSSE Tracker](https://github.com/sermanet/OverFeat)
  - [PyBayes: Recursive Bayesian Filtering](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
  - [FANN" Fast Artificial Neural Network](http://leenissen.dk/fann/wp/)
  - [Faster R-CNN](https://github.com/rbgirshick/fast-rcnn)

## Improved practices 
  - [py.test](http://pytest.org/latest/)
  - [using numba](http://numba.pydata.org/)
  - [Speeding up numpy, weave, cython](http://technicaldiscovery.blogspot.my/2011/06/speeding-up-python-numpy-cython-and.html)
  - [Efficient Overlapping Windows with Numpy](http://www.johnvinyard.com/blog/?p=268)
  
