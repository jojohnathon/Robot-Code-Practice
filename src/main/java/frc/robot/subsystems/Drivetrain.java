package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain implements Subsystem{
    private static Drivetrain instance = null; //singleton stuff
    
    //initialize motors here:
    private static final TalonFX
        leftMaster = Util.createTalonFX(DrivetrainConstants.leftMaster),
        leftSlave = Util.createTalonFX(DrivetrainConstants.leftSlave),
        rightMaster = Util.createTalonFX(DrivetrainConstants.rightMaster),
        rightSlave = Util.createTalonFX(DrivetrainConstants.rightSlave);
    public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(DrivetrainConstants.kMaxSpeedMPS, DrivetrainConstants.kMaxAcceleration);
    public static final ProfiledPIDController LEFT_PID_CONTROLLER = new ProfiledPIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD, constraints);
    public static final ProfiledPIDController RIGHT_PID_CONTROLLER = new ProfiledPIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD, constraints);
    private Drivetrain() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        rightMaster.setInverted(true); //invert right side for tank drive
        rightSlave.setInverted(true);
        register(); //allows this subsytem to work with the command scheduler 
    }

    public static Drivetrain getInstance() {
        if(instance == null) instance = new Drivetrain();
        return instance;
    }

    /*@Override
    public void periodic() { //report or update values to smartdashboard
        SmartDashboard.putNumber("Left Master output", leftMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("Left Slave output", leftSlave.getMotorOutputPercent());
        SmartDashboard.putNumber("Right Master output", rightMaster.getMotorOutputPercent()); 
        SmartDashboard.putNumber("Right Slave output", rightSlave.getMotorOutputPercent());
    }
*/
    public static void setOpenLoop(double left, double right) {
        //refer to example subsystem
        //sets hardware to run at percent of max power
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput,right);
    }
    /*
    public static void setVoltages(double leftv, double rightv) {
        setOpenLoop(leftv/Constants.kMaxVoltage, rightv/Constants.kMaxVoltage);
    }
    */
    public void stop() {
        setOpenLoop(0, 0);
    }

    /*
    public double getEncoderMPS() {
    
        
        return 0;
    }
    */
}
