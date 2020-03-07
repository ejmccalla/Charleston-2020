# LimelightVision Controller

The LimelightVision class implements control algorithms using vision as a direct feedback sensor.

## Features

* Interactive PID Tuning - All control parameters are sent to the *Live Window* output for on-the-fly control loop
                           tuning.
* Threading - Class can be configured to run in the main robot thread or in it's own thread.
* Automated LED Control - The LED's will only be turned on while actively running a targeting command.
* Distance Estimator - Support for measuring distance-to-target using fixed angles and bounding box target area.

## Prerequisites

* [Calibrated Limelight Pipeline(s)](https://docs.limelightvision.io/en/latest/vision_pipeline_tuning.html)<br>
* [Calibrated Limelight Crosshair(s)](https://docs.limelightvision.io/en/latest/crosshair_calibration.html)<br>
* Calibrated Focal Length
* Calibrated Control Loop Parameters

![Shuffleboard LiveWindow Layout](https://github.com/ejmccalla/Charleston-2020/blob/master/images/On-the-Fly_Shuffleboard.jpg)

## Controller

All of the shared controller state can be read using the *GetSharedState* method and accessing the member variables.
The shared state *outputTurn* and *outputDistance* are the raw output from the PIDF calculations.  It is up to the
caller to ensure these outputs are applied correctly to the mechanism that is being controlled.  The feed-forward
term is applied differently by this controller than normal implementations.  The term is only used to set the
*outputTurn* when the Limelight doesn't have a target identified.  When the controller receives an input targeting
comnmand, and a target isn't within sight, the controller will enter into the *searching* state.  In this state,
the *outputTurn* will be set to the feed-forward, the *outputDistance* will be set to 0.0, and the controller will
keep doing so until either a target is identified or the search times out.  For the case when the controller has
identified a target, as indicated by the shared state *foundTarget*, the *outputTurn* and *outputDistance* will be the
normal PID calculations.
