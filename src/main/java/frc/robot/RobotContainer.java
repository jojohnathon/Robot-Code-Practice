package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Drive.State;
import frc.robot.subsystems.*;

public class RobotContainer {
    private static RobotContainer instance = null;
    private static final XboxController driverController = new XboxController(Constants.InputPorts.driverController);
    private static final XboxController operatorController = new XboxController(Constants.InputPorts.operatorController);
    private static final JoystickButton driver_A = new JoystickButton(driverController, 1),
            driver_B = new JoystickButton(driverController, 2), driver_X = new JoystickButton(driverController, 3),
            driver_Y = new JoystickButton(driverController, 4), driver_LB = new JoystickButton(driverController, 5),
            driver_RB = new JoystickButton(driverController, 6), driver_VIEW = new JoystickButton(driverController, 7),
            driver_MENU = new JoystickButton(driverController, 8);
    private static final JoystickButton operator_A = new JoystickButton(operatorController, 1),
    operator_B = new JoystickButton(operatorController, 2), operator_X = new JoystickButton(operatorController, 3),
    operator_Y = new JoystickButton(operatorController, 4), operator_LB = new JoystickButton(operatorController, 5),
    operator_RB = new JoystickButton(operatorController, 6), operator_VIEW = new JoystickButton(operatorController, 7),
    operator_MENU = new JoystickButton(operatorController, 8);
    private static final POVButton operator_DPAD_UP = new POVButton(operatorController, 0),
    operator_DPAD_RIGHT = new POVButton(operatorController, 90), operator_DPAD_DOWN = new POVButton(operatorController, 180),
    operator_DPAD_LEFT = new POVButton(operatorController, 270);

    //TODO: define all subsystems here
    public ExampleSubsystem exampleSubsystem;
    public Drivetrain drivetrain;
    private RobotContainer() {
        //TODO: initialize subsystems here
        exampleSubsystem = ExampleSubsystem.getInstance();
        drivetrain = Drivetrain.getInstance();

        //TODO: define default commands here
        exampleSubsystem.setDefaultCommand(new ExampleCommand());
        drivetrain.setDefaultCommand(new Drive(State.TankDrive));
        //bindOI should be the last thing that runs during construction of RobotContainer
        bindOI();
    }
    

    private void bindOI() {
        //TODO: bind controller inputs to commands/runnables
        /* i.e.
        driver_x.whileHeld(new ExampleCommand(), exampleSubsystem);
            or
        driver_x.whileHeld(new StartEndCommand(() -> exampleSubsystem.setOpenLoop(0.05), exampleSubsystem::stop, exampleSubsystem));
        */
    }

    

    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();
        return instance;
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
    public static double getAltThrottle() {
        return -deadbandX(driverController.getRightY(), Constants.DriverConstants.kJoystickDeadband);
    }

    public static double getTurn() {
        return deadbandX(driverController.getRightX(), Constants.DriverConstants.kJoystickDeadband);
    }

}
