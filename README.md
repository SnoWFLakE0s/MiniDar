# MiniDar
___
## Overview
This is a project using a low-cost LIDAR (the TF-SOLAR-LIDAR07, ~25 USD) for target detection and tracking purposes. Although high-power, high-speed 3-D scanning LIDAR units are commercially available, they are far too expensive for widespread usage at this point in time, and thus a low-cost alternative is preferred. Using a standard mechanical-scanning array setup, a single point LIDAR can provide apt data for the average usage case.

## Parts List
Below are the parts that have been used.

- 1 x TF-SOLAR-LIDAR07 (Available online from retailers)
- 1 x MG90S Servo Motor
- 1 x Arduino Uno or similar MCU
- 1 x ST7789 IPS Display
- 4 x 2.2k Ohm Resistors
- 4 x 3.3k Ohm Resistors
- Adequate amount of wires & breadboard or similar

## Usage
The provided code is built such that customizable setting-variables are near the top of the document. A list of the modifiable settings are below.

* `SRCRNG`: Scanning range of the system - LIDAR module supports up to 12m.
* `RINGS`: How many gridlines are desired along the entire range of scan.
* `PPS`: Resolution of the scanning system. A lower resolution will improve scan speed, while a higher resolution will make it slower but more detailed.

## Interface
A standard PPI for scanning arrays (Plan-Position Indicator) is used to display the collected data from the LRF module. Range data, ring intervals, and scanning FOV is displayed as well.