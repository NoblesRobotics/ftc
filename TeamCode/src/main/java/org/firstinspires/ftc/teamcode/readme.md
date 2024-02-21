# Nobles FTC 2023-24

Guide to the organization of the code:
* findblock
  * BlockFinder: provides boilerplate to allow BlockFinderPipeline to process frames
  * BlockFinderPipeline: machine vision code that identifies the cubes
* opmodes
  * \[Blue | Red]\[Close | Far](Park): auto opmodes, only BlueClose is commented
  * TurnInPlace: teleop opmode
* slide
  * SlideAssembly: contains setpoints for servos and slides
  * DoubleSlide: runs two slides at once with one set of functions
  * ActionQueuer: allows SlideAssembly to run positions sequences asynchronously
* swerve
  * servo
    * SwerveServo: represents a continuous servo with an absolute encoder, runs it to any angle
    * SwerveServoStorage: contains static variables that store the calibrated state of the servos
    * ClearServoCache: erases SwerveServoStorage, must be run if servos have been moved by hand
  * SwerveModule: represents a servo and motor, runs at any angle at any power in the most efficient possible way
  * SwerveDrive: base class that represents four swerve modules, contains utility functions
  * AutoSwerveDrive: autonomous swerve driving
  * TeleOpSwerveDrive: teleop swerve driving
  * PIDImpl: implementation of a PID controller
  * SimpleIMU: allows easy access to onboard gyroscope
* test: various opmodes for calibrating setpoints, testing autonomous driving, testing camera
* AprilTagFinder: uses FTC Vision Portal to find April Tags
* AttributeTelemetry: allows access to telemetry object in all code using static fields
* SinglePress: used to call a function only at the moment a button is pressed