# YOLOv3-tracking

> Kalman filter object tracker base on YOLOv3

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

This repo develops a image-based object tracker which based on YOLOv3 object detector. The repo runs on ROS middleware which is popular to use in robotics field.


The ROS version YOLOv3 is implemented by [leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros) which given the camera image then publish the detection bounding box.
This project would subscribe the detection result and use kalman filter the track those objects. The object which is tracked would have same track id.

[![Demo Video](https://drive.google.com/file/d/121_VBHeTh1A8GS49ih5nXeVw9z2b02Kd/view?usp=sharing)](https://drive.google.com/file/d/1-SRf17Lk1IWu7cImY4xdLh0LEr4j_ylM/view?usp=sharing)
In the demo video, the right window is the YOLO detection result, and the left window is the tracking result. The track id represents as bounding box color.



## Installation

###  Building
The way to build darknet_ros could refer to [leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros)

The way to build kalman filter tracker:
```
cd catkin_workspace
git clone --recursive https://github.com/Abekabe/YOLOv3-tracking.git
cd ../
catkin_make
```


## Usage example

Make sure the darknet_ros package is running and the bounding_boxes topic is published before start tracking.

```
source devel/setup.sh
rosrun object_tracking tracking
```


## Release History

* 1.0.1
    * The first proper release
    * The object tracker works now
* 0.0.1
    * Work in progress

## Meta

Caleb Lee - caleb.eed04@nctu.edu.tw


Distributed under the BSD license. See `LICENSE` for more information.

## Contributing

1. Fork it (<https://github.com/Abekabe/YOLOv3-tracking/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some new staff'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Abekabe/YOLOv3-tracking?style=for-the-badge
[contributors-url]: https://github.com/Abekabe/YOLOv3-tracking/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Abekabe/YOLOv3-tracking?style=for-the-badge
[forks-url]: https://github.com/Abekabe/YOLOv3-tracking/network/members
[stars-shield]: https://img.shields.io/github/stars/Abekabe/YOLOv3-tracking?style=for-the-badge
[stars-url]: https://github.com/Abekabe/YOLOv3-tracking/stargazers
[issues-shield]: https://img.shields.io/github/issues/Abekabe/YOLOv3-tracking?style=for-the-badge
[issues-url]: https://github.com/Abekabe/YOLOv3-tracking/issues
[license-shield]: https://img.shields.io/github/license/Abekabe/YOLOv3-tracking?style=for-the-badge
[license-url]: https://github.com/Abekabe/YOLOv3-tracking/blob/master/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/chia-le-lee-ba0bbb180/
[product-screenshot]: mages/screenshot.png
