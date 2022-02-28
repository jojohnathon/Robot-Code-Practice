package frc.robot;

import frc.robot.Constants;
import frc.robot.Autonomous.Auto;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveXMeters;
import frc.robot.commands.HubTrack;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnXDegrees;
import frc.robot.commands.CargoTrack;
import frc.robot.commands.ConveyorQueue;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

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

    public static NetworkTable limelightIntake;
    public static NetworkTable limelightShooter;

    public static Drivetrain drivetrain;
    public static Intake intake;
    public static Climber climber;
    public static Arm arm;
    public static Shooter shooter;
    public static Conveyor conveyor;
    public static ColorSensorV3 colorSensorV3;
    public static AHRS navX; 
    public static PhotonCamera camera;
    private RobotContainer() {
        /*camera = new PhotonCamera("photonvision");*/
        navX = new AHRS(Port.kMXP);
        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(Drive.State.CheesyDriveOpenLoop));
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        climber = Climber.getInstance();
        shooter = Shooter.getInstance();
        conveyor = Conveyor.getInstance();
        conveyor.setDefaultCommand(new ConveyorQueue());
        colorSensorV3 = Util.createColorSensorV3(ConveyorConstants.colorSensorV3);
        limelightIntake = NetworkTableInstance.getDefault().getTable("limelight-intake");
        limelightShooter = NetworkTableInstance.getDefault().getTable("limelight-shooter");

        bindOI();
    }

    public static Command getAutonomousCommand() {
        Command auto = new SequentialCommandGroup(Auto.getShootCommand(), Auto.getBackupCommand(), Auto.getIntakeCommand());
        switch(DriverStation.getLocation()) { //TODO: change how auto functions based on our team's starting position on the field
            case 1:
                auto = auto.andThen();
                return auto;
            case 2:
                auto = auto.andThen();
                return auto;
            case 3:
                auto = auto.andThen();
                return auto;
            default:
                return auto;
        }
    }

    
    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();
        return instance;
    }

    private void bindOI() {
        driver_RB.whileHeld(new RunCommand(()->arm.setGoal(Arm.State.OUT), arm)
                    .alongWith(new RunCommand( ()->intake.intake(0.5)))
                    .alongWith(new RunCommand( ()->intake.setConveyor(0.5))))
                .whenReleased(new RunCommand( ()->arm.setGoal(Arm.State.STORED), arm)
                    .alongWith(new InstantCommand(intake::stopIntake)));
        /*driver_LB.whileHeld(new Shoot(0.65));*/
        driver_X.whileHeld(new HubTrack());
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
        if(Math.abs(input) <= deadband) {
            return 0;
        } else if(Math.abs(input) == 1) {
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

    /*public static Color getColor() {
        return colorSensorV3.getColor();
    }     

    public int getProximity() {
        return colorSensorV3.getProximity();
    }*/

    /**
     * Set the LED mode on the Limelight
     * 
     * @param ledMode The mode to set the Limelight LEDs to
     */
    public void setIntakeLEDMode(LEDMode ledMode) {
        limelightIntake.getEntry("ledMode").setNumber(ledMode.val);
    }
    public void setShooterLEDMode(LEDMode ledMode) {
        limelightShooter.getEntry("ledMode").setNumber(ledMode.val);
    }
    /**
     * Sets the appearance of the Limelight camera stream
     * 
     * @param stream Stream mode to set the Limelight to
     */
    public void setIntakeStreamMode(StreamMode stream) {
        limelightIntake.getEntry("stream").setNumber(stream.val);
    }
    public void setShooterStreamMode(StreamMode stream) {
        limelightShooter.getEntry("stream").setNumber(stream.val);
    }
    /**
     * Sets Limelight vision pipeline
     * 
     * @param pipeline The pipeline to use
     */
    public void setIntakePipeline(IntakeVisionPipeline pipeline) {
        limelightIntake.getEntry("pipeline").setNumber(pipeline.val);
    }
    public void setShooterPipeline(ShooterVisionPipeline pipeline) {
        limelightShooter.getEntry("pipeline").setNumber(pipeline.val);
    }
    /**
     * Returns the horizontal offset between the target and the crosshair in degrees
     * 
     * @return the horizontal offset between the target and the crosshair in degrees
     */
    public static double getIntakeXOffset() {
        return -limelightIntake.getEntry("tx").getDouble(0);
    }
    public static double getShooterXOffset() {
        return -limelightShooter.getEntry("tx").getDouble(0);
    }
    /**
     * Returns the vertical offset between the target and the crosshair in degrees
     * 
     * @return the vertical offset between the target and the crosshair in degrees
     */
    public static double getIntakeYOffset() {
        return -limelightIntake.getEntry("ty").getDouble(0.0);
    }
    public static double getShooterYOffset() {
        return -limelightShooter.getEntry("ty").getDouble(0.0);
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

    public enum IntakeVisionPipeline {
        RED(0), BLUE(1), DRIVER(2), INVALID(3);

        public int val;

        IntakeVisionPipeline(int val) {
            this.val = val;
        }
        
    }
    public enum ShooterVisionPipeline {
        ROBOT(0);

        public int val;

        ShooterVisionPipeline(int val) {
            this.val = val;
        }
        
    }
    public static IntakeVisionPipeline allianceToPipeline() {
        Alliance alliance = DriverStation.getAlliance();
        switch(alliance) {
            case Blue:
                return IntakeVisionPipeline.BLUE;
            case Red:
                return IntakeVisionPipeline.RED;
            default:
                return IntakeVisionPipeline.INVALID;
        }
    }
    /*public static PhotonPipelineResult getSnapshot() {
        return camera.getLatestResult();
    }*/
}
