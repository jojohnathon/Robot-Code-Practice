package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Units;
import frc.robot.Util;
import frc.robot.Constants.ExampleConstants;

public class ExampleSubsystem implements Subsystem {
    //Initialize motors here or in the constructor (you will always use some sort of motor controller, such as TalonFX for Falcons, or CANSparkMax for NEO motors)
    //private TalonFX exampleTalon = Util.createTalonFX(ExampleConstants.exampleTalon); For a Falcon motor on a port defined in Constants.java
    //Example motor code is commented out because this project is meant to be deployable to our 2022 season robot, with these examples serving as a starter for what to do, and these motors do not exist on the robot
    private static ExampleSubsystem instance = null; //Necessary for singleton implementation
    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(ExampleConstants.kS, ExampleConstants.kV, ExampleConstants.kA);
    public static final PIDController posController = new PIDController(ExampleConstants.kP, ExampleConstants.kI, ExampleConstants.kD);
    public static final PIDController velController = new PIDController(ExampleConstants.kPv, ExampleConstants.kIv, ExampleConstants.kDv);

    private ExampleSubsystem() {
        //exampleTalon.setInverted(false); An example of extra hardware configuration specific to this motor, which we generally do here in the constructor

        register(); //Necessary for the subsystem to work with the Command-Based framework
    }
    public static ExampleSubsystem getInstance() { //Gatekeeps creation of ExampleSubsystem to ensure only 1 instance is ever created
        if(instance == null) instance = new ExampleSubsystem();
        return instance;
    }

    @Override
    public void periodic() { //Periodic runs every 20ms with the Command Scheduler, generally we use periodic to report or update measured values
        SmartDashboard.putNumber("Example reported value here", 0); 
    }

    /*
        Sets the example Talon to run at a percent of max power
        @param percentValue a value from -1.0 to 1.0 representing the percent of max power to supply, with negative values corresponding to spinning in a different direction
    */
    public void setOpenLoop(double percentValue) {
        //exampleTalon.set(ControlMode.PercentOutput, percentValue);
    }

    public void stop() {
        setOpenLoop(0.0);
    }

    public double getEncoderMPS() {
        //return Units.DrivetrainUnits.TicksToMeters(exampleTalon.getSelectedSensorVelocity()); The built in encoder measures units of rotation in ticks, so we have a method of unit conversion into a human-understandable format
        return 0;
    }
}
