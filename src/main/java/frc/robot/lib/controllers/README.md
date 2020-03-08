# LimelightVision Controller

The LimelightVision class implements control algorithms using vision as a direct feedback sensor.

## Features

* Interactive PID Tuning - All control parameters are sent to the *Live Window* output for on-the-fly control loop
                           tuning.
* Threading - Controller can be configured to run in the main robot thread or in it's own thread.
* Automated LED Control - The LED's will only be turned on while actively running a targeting command.
* Distance Estimator - Support for measuring distance-to-target using fixed angles and target bounding box.

## Prerequisites

* [Calibrated Limelight Pipeline(s)](https://docs.limelightvision.io/en/latest/vision_pipeline_tuning.html)<br>
* [Calibrated Limelight Crosshair(s)](https://docs.limelightvision.io/en/latest/crosshair_calibration.html)<br>
* Calibrated Focal Length
* Calibrated Control Loop Parameters

## Shuffleboard Layout

![Shuffleboard LiveWindow Layout](https://github.com/ejmccalla/Charleston-2020/blob/master/images/On-the-Fly_Shuffleboard.jpg)