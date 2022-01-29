package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor implements Subsystem {
    private static CANSparkMax master;
    private static Conveyor instance;
    private static final DigitalInput intakePhotoelectric = new DigitalInput(ConveyorConstants.intakePhotoelectric); //sensor closest to intake
    private static final DigitalInput shooterPhotoelectric = new DigitalInput(ConveyorConstants.shooterPhotoelectric); //sensor closest to shooter
    public static Conveyor getInstance() {
        if (instance == null) instance = new Conveyor();
        return instance;
    }

    private Conveyor() {
        master = Util.createSparkMAX(ConveyorConstants.motor, MotorType.kBrushless);
        master.setInverted(true);
        master.burnFlash();
        register();
    }

    public void setOpenLoop(double value) {
        master.set(value);
    }

    public void stop() {
        master.stopMotor();
    }

    public boolean getIntakeSensor() {
        return !intakePhotoelectric.get();
    }

    public boolean getShooterSensor() {
        return !shooterPhotoelectric.get();
    }
}