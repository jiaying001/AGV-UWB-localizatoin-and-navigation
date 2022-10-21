# AGV-UWB-localizatoin-and-navigation
This is a ROS package for indoor agv localization and navigation based on DW1000 UWB transceiver. Laser sensor is used for obstacles avoidance. 

这是一个基于UWB无人车室内定位的ROS包。 用到了激光雷达作为避障检测。
## Hardware
- D-DWM-PG2.5 UWB transceiver developed from decaWave DW1000 chip

- UP boards

- AGV (Pioneer3AT mobile robot)

- Lidar (Hokuyo scanning laser UTM-30LX)

## Overall system architecture block diagram

Installation of UWB coordinating system on ROS across two UP boards

![Overall system architecture block diagram](https://github.com/jiaying001/AGV-UWB-localizatoin-and-navigation/blob/main/images/Overall%20system%20architecture%20block%20diagram.png)

Hardware setup on Pioneer3AT mobile robot

![Hardware setup on Pioneer3AT mobile robot](https://github.com/jiaying001/AGV-UWB-localizatoin-and-navigation/blob/main/images/Hardware%20setup%20on%20Pioneer3AT%20mobile%20robot.png)

## Usage

Potential field method

![potential_field_method](https://github.com/jiaying001/AGV-UWB-localizatoin-and-navigation/blob/main/images/potential_field_method.png)

UWB target pathtracking

![uwb_target_pathtracking](https://github.com/jiaying001/AGV-UWB-localizatoin-and-navigation/blob/main/images/uwb_target_pathtracking.png)
