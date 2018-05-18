<img src="https://storage.googleapis.com/ultralytics/UltralyticsLogoName1000×676.png" width="200">  

# Introduction

This directory contains software developed by Ultralytics LLC. For more information on Ultralytics projects please visit:
http://www.ultralytics.com  

# Description

The https://github.com/ultralytics/kinect repo contains 3D scene reconstruction algorithms applied to data collected from the  Microsoft Kinect depth-image sensor.  More example videos available at https://www.youtube.com/ultralytics.

[![IMAGE ALT TEXT](https://github.com/ultralytics/kinect/blob/master/preview.jpg)](https://www.youtube.com/watch?v=qTAWyXwABos "Kinect Video")

# Requirements

MATLAB >= 2018a with the following toolbox installed:  

- ```Statistics and Machine Learning Toolbox```
- ```Signal Processing Toolbox```

Ultralytics MATLAB common functions must also be cloned and added to the MATLAB path:

1. From shell: ```$ git clone https://github.com/ultralytics/matlab-common```
2. From MATLAB: ```>> apppath(genpath('/matlab-common'))```

# Running
From MATLAB: ```>> buildscene.m```

# Contact

For questions or comments please contact Glenn Jocher at glenn.jocher@ultralytics.com or visit us at http://www.ultralytics.com/contact
