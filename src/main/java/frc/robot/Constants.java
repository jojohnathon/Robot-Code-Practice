package frc.robot;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Units;

//Hello
public class Constants {
    public static final double dt = 0.02;
    public static final double kMaxVoltage = 12.0;

    public static class ExampleConstants {
        public static final int exampleTalon = 1;

        /* Feedforward Constants */
        public static double kS = 0.1;
        public static double kV = 1.0;
        public static double kA = 0.01;

        /* Positional PID Constants */
        public static double kP = 0.1;
        public static double kI = 0.05;
        public static double kD = 0; //Typically, for a positional PID controller (i.e. setpoint is in terms of a position, as in turn motor to move 5 meters), just P and I terms are sufficient for error correction

        /* Velocity PID Constants */
        public static double kPv = 0.07;
        public static double kIv = 0;
        public static double kDv = 0.1; //Typically, for a velocity PID controller (i.e. setpoint is in terms of a velocity, as in maintain a velocity of 1 meter/s), P and D are used to maintain desired speed, while the I term is not used 
    }

    public static class InputPorts {
        public static final int driverController = 0, operatorController = 1;
    }

    public static class DriverConstants {
        /* Common drive mode settings */
        public static final double kJoystickDeadband = 0.07; // How much of joystick is "dead" zone [0,1]
        public static final double kDriveSens = 0.5; // Overall speed setting (turn down for demos) [0,1]
        public static final double kTurnInPlaceSens = 0.3; // Maximum turn-in-place rate (in percent of max) to allow
                                                            // robot to turn to [0,1]
        public static final double kTurnSens = 0.7; // Maximum normal turning rate (in percent of max) to allow robot to
                                                  // turn to [0,1]
    }
    
    public static class ArmConstants {
        public static final int actuateMotor = 4; //change to actuatemotor

        /* PID Constants */
        public static double kP = 2.9;
        public static double kI = 0;
        public static double kD = 0;

        /* Feedforward Constants UNTESTED*/
        public static double kS = 0.402;
        public static double kCos = 0.771;
        public static double kV = 0.758;
        public static double kA = 0.00717;

        /* Intake constants UNTESTED*/
        public static double kMaxVelocity = 0.25; // Maximum velocity to turn arm at, radians per second
        public static double kMaxAcceleration = 2; // Maximum acceleration to turn arm at, radians per second per second
        public static double kArmOffset = Math.toRadians(27); //The extension of the arm necessary to enter "intake" position

    }

    public static class DrivetrainConstants {
        public static final int
        /* Drivetrain motor IDs */ 
            leftMaster = 3, // TalonFX right Masters & Slaves currently reversed
            leftSlave = 1, // TalonFX
            rightMaster = 2, // TalonFX
            rightSlave = 4; // TalonFX
        
        /* feedforward constants UNTESTED*/
        public static final double kS = 0.364; // voltage required to overcome friction (V)
        public static final double kV = 2.34; // voltage over velocity (V/(meters/second))
        public static final double kA = 0.0824; // voltage over acceleration (V(meters/second/second))

        /* PID constants */
        public static final double kP = 2.9;
        public static final double kI = 0;
        public static final double kD = 0;

        /* Wheels Constants */
        public static final double kTicksPerRotation = 2048 * 10.71; // Falcon 500 integrated encoder (2048 CPR)
                                                                     // multiplied by gear ratio (10.71:1)
        public static final double kWheelDiameter = Units.InchesToMeters(6);

        public static final double kMaxSpeedMPS = 4.972; // max speed in meters per second
        public static final double kMaxAcceleration = 0; //max acceleration in meters per second per second
        public static final double kTrackWidth = 0.7051868402911773; // distance between wheels
        public static final double kMaxCurvature = Math.toRadians(-162); // Maximum turn rate in radians per meter

        public static final double sdx = 0.2;

        public static final double kPV = 0;
    }

    public static class IntakeConstants {
        /* Motors */
        public static final int rollerMotor = 3;
    }

    public static class ShooterConstants {
        /* PID constants */
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0; 

         /* Feedforward Constants */ 
         public static double kS = 0.17205;
         public static double kV = 0.40335;
         public static double kA = 0.016457;
 
         /* Shooter Constants */ 
        public static double wheelDiameter = Units.InchesToMeters(4);
        public static double maxVelRadS = 70.85 * 2 * Math.PI; //estimated max speed in radians/sec
        public static double kTolerance = 0;
        public static int master = 1;
        public static int slave = 10;
    }
    public static class ConveyorConstants {
        public static final int conveyorMotor = 5;
        public static final Port colorSensorV3 = Port.kOnboard;
        public static final double minimumProximity = 1800;
        public static final int motor = 2;
        public static final int intakePhotoelectric = 1;
        public static final int shooterPhotoelectric = 3;

    }
    public static class ClimbConstants {
        public static final int rightMotor = 6, leftMotor = 5;
        /* PID constants */
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static class VisionConstants {
        /*
            Servo Constants
        */
        //public static final int servoChannel = 0;

        /* Color Sensor Constants */
        public static final int minimumProximity = 450;
        public static final double minimumSimilarity = 0.6;

        /* Turn PID Constants */
        public static double kPTurn = 0.1;
        public static double kITurn = 0;
        public static double kDTurn = 0; 
        public static double kTurnTolerance = 1;

        /* Distance PID Constants */
        public static double kPDist = 0.1;
        public static double kIDist = 0;
        public static double kDDist = 0;
        public static double kDistTolerance = 0;
    }
}
