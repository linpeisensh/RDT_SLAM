# stereo_ptam

This python project is a complete implementation of Stereo PTAM, based on C++ project [lrse/sptam](https://github.com/lrse/sptam) and paper "[S-PTAM: Stereo Parallel Tracking and Mapping](http://webdiis.unizar.es/~jcivera/papers/pire_etal_ras17.pdf) Taihu Pire et al. RAS17", with some modifications.   

> S-PTAM is a Stereo SLAM system able to compute the camera trajectory in real-time. It heavily exploits the parallel nature of the SLAM problem, separating the time-constrained pose estimation from less pressing matters such as map building and refinement tasks. On the other hand, the stereo setting allows to reconstruct a metric 3D map for each frame of stereo images, improving the accuracy of the mapping process with respect to monocular SLAM and avoiding the well-known bootstrapping problem. Also, the real scale of the environment is an essential feature for robots which have to interact with their surrounding workspace.   

S-PTAM system overview (from [S-PTAM paper](http://webdiis.unizar.es/~jcivera/papers/pire_etal_ras17.pdf) page 11):  
![](imgs/sptam_overview.png)

As stated in the [S-PTAM paper](http://webdiis.unizar.es/~jcivera/papers/pire_etal_ras17.pdf) (page 39), S-PTAM's results on KITTI dataset is comparable to stereo version of [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), and better than stereo LSD-SLAM. It's very inspiring.


## Features 
(of this implementation)
* Semantic Segmentation of Moving Objects
* Multithreads Tracking, Mapping, and Loop Closing;
* Covisibility Graph (representing the relation between keyframes, mappoints and measurements);
* Local Bundle Adjustment and Pose Graph Optimization;
* Motion Model (used for pose prediction, then for reliable feature matching);
* Data loader for datasets [KITTI Odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) and [EuRoC MAV](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets);


## Requirements
* Python 3.6
* numpy
* opencv-python, opencv-contrib-python  == 4.1.2.30
* [g2o](https://github.com/uoip/g2opy) <sub>(python binding of C++ library [g2o](https://github.com/RainerKuemmerle/g2o))</sub> for optimization
* apex
* cocoapi
* torchvision == 0.2.1
* torch == 1.2.0
* maskrcnn_benchmark

## Usage
`python dysptam.py --device cpu/cuda --no-viz --dataset kitti --path path/to/your/KITTI_odometry_dataset/sequences/00`  



### TODO:
Exhaustive evaluation on datasets. (There seems to be a python package [MichaelGrupp/evo](https://github.com/MichaelGrupp/evo) for odometry/SLAM algorithm evaluation)

## License
This python reimplementation is largely based on [sptam](https://github.com/lrse/sptam), so it's licensed under GPLv3 License. 

## Contact
If you have problems related to the base S-PTAM algorithm, you can contact original authors [lrse](https://github.com/lrse) (robotica@dc.uba.ar), or refer to the related papers:  
[1]  Taihú Pire,Thomas Fischer, Gastón Castro, Pablo De Cristóforis, Javier Civera and Julio Jacobo Berlles.
**S-PTAM: Stereo Parallel Tracking and Mapping**
Robotics and Autonomous Systems, 2017.  

[2] Taihú Pire, Thomas Fischer, Javier Civera, Pablo De Cristóforis and Julio Jacobo Berlles.  
**Stereo Parallel Tracking and Mapping for Robot Localization**  
Proc. of The International Conference on Intelligent Robots and Systems (IROS), Hamburg, Germany, 2015.  


This project is based on https://github.com/uoip/stereo_ptam.
