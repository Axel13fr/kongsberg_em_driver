# kongsberg_em_driver [![Build Status](https://travis-ci.com/Axel13fr/kongsberg_em_driver.svg?branch=master)](https://travis-ci.com/Axel13fr/kongsberg_em_driver)
A ROS implementation for communicating with Kongsberg multibeam echo sounder series.

This implementation is largely based on the work done by WHOI Deep Submergence Lab on this repository: https://bitbucket.org/whoidsl/ds_kongsberg/

## Dependencies

See dependencies.rosinstall

- [ds_kongsberg_msgs](https://github.com/Axel13fr/ds_kongsberg_msgs): separated from the existing [ds package](https://bitbucket.org/whoidsl/ds_kongsberg/master) to avoid compiling the original implementation.

- [ds_msgs](https://bitbucket.org/whoidsl/ds_msgs/src/master/)

- [lib_rosasio](https://github.com/Axel13fr/librosasio) (combines ROS and Boost Asio event loops)