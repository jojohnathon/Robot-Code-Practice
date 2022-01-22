package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotContainer {
    private static RobotContainer instance = null;
    private static final XboxController driverController = new XboxController(Constants.InputPorts.driverController);
    private static final XboxController operatorController = new XboxController(Constants.InputPorts.operatorController);
    private static final JoystickButton driver_A = new JoystickButton(driverController, 1),
            driver_B = new JoystickButton(driverController, 2), driver_X = new JoystickButton(driverController, 3),
            driver_Y = new JoystickButton(driverController, 4), driver_LB = new JoystickButton(driverController, 5),
            driver_RB = new JoystickButton(driverController, 6), driver_VIEW = new JoystickButton(driverController, 7),
            driver_MENU = new JoystickButton(driverController, 8);

    private static NetworkTable limelight;

    public static Drivetrain drivetrain;
    public static Intake intake;
    public static Climber climber;
    public static Arm arm;
    public static ColorSensorV3 colorSensorV3;

    private RobotContainer() {
        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(Drive.State.CheesyDriveOpenLoop));
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        climber = Climber.getInstance();
        colorSensorV3 = Util.createColorSensorV3(VisionConstants.colorSensorV3);

        limelight = NetworkTableInstance.getDefault().getTable("limelight");

        bindOI();
    }
    
    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();
        return instance;
    }

    private void bindOI() {
        driver_RB.whileHeld(new RunCommand(()->arm.rotate(-0.4), arm)
                    .alongWith(new RunCommand( ()->intake.intake(0.5)))
                    .alongWith(new RunCommand( ()->intake.setConveyor(0.5))))
                .whenReleased(new RunCommand( ()->arm.rotate(0.35), arm)
                    .alongWith(new InstantCommand(intake::stopIntake)));
    }

     /**
     * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
     * linear from (deadband, 0) to (1,1)
     * 
     * @param input    The input value to rescale
     * @param deadband The deadband
     * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
     */
    public static double deadbandX(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

    public static double getThrottle() {
        return -deadbandX(driverController.getLeftY(), Constants.DriverConstants.kJoystickDeadband);
    }

    public static double getTurn() {
        return deadbandX(driverController.getRightX(), Constants.DriverConstants.kJoystickDeadband);
    }

    public static class RGBValues {
        private int red;
        private int green;
        private int blue;
        public RGBValues(int r, int g, int b) {
            red = r;
            green = g;
            blue = b;
        }
        public int getR() {
            return red;
        }
        public int getG() {
            return green;
        }
        public int getB() {
            return blue;
        }
    }

    public RGBValues getColor() {
        return new RGBValues(colorSensorV3.getRed(), colorSensorV3.getGreen(), colorSensorV3.getBlue());
    }     

    public int getProximity() {
        return colorSensorV3.getProximity();
    }

    /**
     * Set the LED mode on the Limelight
     * 
     * @param ledMode The mode to set the Limelight LEDs to
     */
    public void setLEDMode(LEDMode ledMode) {
        limelight.getEntry("ledMode").setNumber(ledMode.val);
    }

    /**
     * Sets the appearance of the Limelight camera stream
     * 
     * @param stream Stream mode to set the Limelight to
     */
    public void setStreamMode(StreamMode stream) {
        limelight.getEntry("stream").setNumber(stream.val);
    }

    /**
     * Sets Limelight vision pipeline
     * 
     * @param pipeline The pipeline to use
     */
    public void setPipeline(VisionPipeline pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline.val);
    }

    /**
     * Returns the horizontal offset between the target and the crosshair in degrees
     * 
     * @return the horizontal offset between the target and the crosshair in degrees
     */
    public static double getXOffset() {
        return -limelight.getEntry("tx").getDouble(0);
    }

    /**
     * Returns the vertical offset between the target and the crosshair in degrees
     * 
     * @return the vertical offset between the target and the crosshair in degrees
     */
    public static double getYOffset() {
        return -limelight.getEntry("ty").getDouble(0.0);
    }

    /**
     * Enum representing the different possible Limelight LED modes
     */
    public enum LEDMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        public int val;

        LEDMode(int val) {
            this.val = val;
        }
    }

    /**
     * Enum representing the different possible Limelight stream modes
     */
    public enum StreamMode {
        SIDE_BY_SIDE(0), PIP_MAIN(1), PIP_SECONDARY(2);

        public int val;

        StreamMode(int val) {
            this.val = val;
        }
    }

    /**
     * Enum representing the different possible Limelight vision pipelines
     */
    public enum VisionPipeline {
        VISION(0), DRIVER(1);

        public int val;

        VisionPipeline(int val) {
            this.val = val;
        }
    }
}
