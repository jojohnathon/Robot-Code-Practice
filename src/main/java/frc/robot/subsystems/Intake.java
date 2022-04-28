package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake implements Subsystem{
    private static final CANSparkMax rollerMotor = Util.createSparkMAX(IntakeConstants.rollerMotor, MotorType.kBrushless);
    private static final CANSparkMax conveyorMotor = Util.createSparkMAX(ConveyorConstants.conveyorMotor, MotorType.kBrushless);
    private static final CANSparkMax armMotor = Util.createSparkMAX(ArmConstants.actuateMotor, MotorType.kBrushless);

    private RelativeEncoder armEncoder = armMotor.getEncoder();

    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }

    private void Arm() { //check why this method has to be void and Intake() doesn't
        armMotor.setInverted(false);
        register();
    }

    public void setArm(double value) {
        armMotor.set(value);
    }

    public void resetEncoders() {
        armEncoder.setPosition(0.0);
    }
    
    public double getMeasurement() {
        return armEncoder.getPosition() * (2 * Math.PI);
    }
    private Intake(){
        conveyorMotor.setInverted(true);
        rollerMotor.setInverted(false);
        register();
    }

    public void stopArm() {
        armMotor.set(0);
    }

    public void intake(double value) {
        rollerMotor.set(value);
    }

    public void setConveyor(double value) {
        conveyorMotor.set(value);
    }

    public void stopIntake() {
        rollerMotor.set(0);
        conveyorMotor.set(0);
    }

    public void stopConveyor() {
        conveyorMotor.set(0);
    }


}
