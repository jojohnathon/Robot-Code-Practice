<a href="http://millsroboticsteam253.com/"><img src="https://img.shields.io/badge/BobaBots-253-blue"></img></a>
[![Build Status](https://travis-ci.org/MillsRoboticsTeam253/Code2020.svg?branch=master)](https://travis-ci.org/MillsRoboticsTeam253/Code2020)
# Mills Boba Bots 253 2022 Code
- [x] 2022 WPILib Libraries 

## About
This robotic code is made as a starting template for programming our Rapid React robot. 

Members of the programming department are to fork this repository to their own account and write their own code for the 2022 robot! Examples/templates for writing subsystems and commands are provided, and members are encouraged to use the resources at their disposal to create a working project.

## Timeline/Suggested Order of Priorities
**Driving** <br> 
Since absolutely no hardware has been initialized in this template, members are encouraged to first write a drivetrain subsystem, test that it works, and then write a default drive command to allow driving of the robot. It is suggested that all subsystems are held as fields in RobotContainer.java, initialized in RobotContainer's constructor, and that default commands are defined after all of the subsystems.

**Intake/ejection** <br>
After driving, now you may want to start shooting cargo, right? Well, you need to get cargo first. And for that, logic has to be written for intake. You will need to control arm actuation and retraction, along with running both the roller motor and the conveyor motor for effective intake. You may want to divide these motors into more than one subsystem, as intake requires the conveyor motor, in addition to the arm actuation and arm rollers, but shooting will also eventually require the conveyor motor, in addition to the kicker wheel and flywheel. You will probably want to bind intake and outtake to buttons or bumpers on an Xbox controller. Once you have working intake, quickly write similar code for outtake/ball ejection by keeping arm actuation the same but reversing the roller and conveyor.

**Shooting** <br>
So you can drive and intake. Great! Onwards to shooting. You will probably find implementing shoot to be easier than intake, after all, you simply need to run the flywheel, and use the conveyor and kicker to feed cargo into the flywheel. You have options here: you can either shoot with a command bound to a button/bumper that brings the flywheel up to speed before activating the conveyor and kicker, or you can have a default command run the flywheel constantly and have the kicker/conveyor motors run immediately when the shoot button/bumper is pressed.

**Climb** <br>
Optional, and testing climb after competition will likely be limited. Implementation would be relatively simple: just run 2 motors lol.

**Refining** <br>
Did you implement previous functions with percentages and open loops? Try using feedforward or PID! Not all subsystems need FF or PID to be effective, however, and if you cannot find existing constants for the coefficient terms, don't sweat it! Just refine whatever code you can find constants for.

**Auto** <br>
So your teleop is pretty much good to go. Why not experiment with autonomous? It is suggested to try writing simple routines first, like driving in straight lines, turning in place, shooting, and maybe even attempt intake during auto! For experienced members, here is your time to try out more advanced features! Want to use the limelight to track upper hub? Use the navX for odometry and trajectory following? The stage is yours.
