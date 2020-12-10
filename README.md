# kongsberg_em_driver [![Build Status](https://travis-ci.com/Axel13fr/kongsberg_em_driver.svg?branch=master)](https://travis-ci.com/Axel13fr/kongsberg_em_driver)
A ROS implementation for communicating with Kongsberg multibeam echo sounder series.

This implementation is largely based on the work done by WHOI Deep Submergence Lab on this repository: https://bitbucket.org/whoidsl/ds_kongsberg/

## Dependencies

See .travis.rosinstall file

- [ds_kongsberg_msgs](https://github.com/Axel13fr/ds_kongsberg_msgs): separated from the existing [ds package](https://bitbucket.org/whoidsl/ds_kongsberg/master) to avoid compiling the original implementation.

- [ds_msgs](https://bitbucket.org/whoidsl/ds_msgs/src/master/)

- [lib_rosasio](https://github.com/Axel13fr/librosasio) (combines ROS and Boost Asio event loops)

## Displaying data on RVIZ

The driver publishes a PointCloud2 topic containing the sonar data relative to the Vessel coordinate system (X towards vessel heading, Y starboard, Z down). To display the point cloud over time, the different vessel positions are provided by the ```~mrz_topic``` which contains latitude, longitude and heading. Using it to boardcast a tf between a fixed world coordinate system and the vessel position (after converting lat/long into a cartesian coordinate system) can then be done for RVIZ:

![Example of replayed multibeam point cloud on RVIZ](https://github.com/Axel13fr/kongsberg_em_driver/tree/master/img/sonar_data.png)


