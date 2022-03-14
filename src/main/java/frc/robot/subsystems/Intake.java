package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ConveyorConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake implements Subsystem {
    
    private static final CANSparkMax spinMotor = Util.createSparkMAX(IntakeConstants.spinMotor, MotorType.kBrushless);
    //private static CANSparkMax conveyorMotor;
    private static final CANSparkMax conveyorMotor = Util.createSparkMAX(IntakeConstants.conveyorMotor, MotorType.kBrushless);
    
    private static final DigitalInput intakePhotoelectric = new DigitalInput(ConveyorConstants.intakePhotoelectric); //sensor closest to intake
    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }

    private Intake(){
        //conveyorMotor = Util.createSparkMAX(ConveyorConstants.motor, MotorType.kBrushless);
        conveyorMotor.setInverted(true);
        conveyorMotor.burnFlash();
        /*conveyorMotor.setInverted(false);
        conveyorMotor.burnFlash();*/
        register();
    }

    /**
     * Sets the conveyor to spin at a percent of max speed
     * @param value Percent speed
     */
    /*public void setConveyor(double value) {
        conveyorMotor.set(value);
    }*/

    /**
     * Sets the intake to spin at a given voltage
     * 
     * @param value Percent of maximum voltage to send to motor
     */
    public void intake(double value) {
        spinMotor.set(value);
    }
    public void setConveyor(double value) {
        conveyorMotor.set(value);
    }
    
    /**
     * Stops the intake
     */
    public void stopIntake() {
        spinMotor.set(0);
        conveyorMotor.set(0);
    }

    public void stopConveyor(){
        conveyorMotor.set(0);
    }
    
    public boolean getIntakeSensor() {
        if(!Robot.useV3()) {
            return !intakePhotoelectric.get();
        } else {
            return (RobotContainer.colorSensorV3.getProximity() >= ConveyorConstants.minimumProximity);
        }
    }
}
