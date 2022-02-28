package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor implements Subsystem {
    private static CANSparkMax conveyorMotor, storageMotor;
    private static Conveyor instance;
    private static final DigitalInput intakePhotoelectric = new DigitalInput(ConveyorConstants.intakePhotoelectric); //sensor closest to intake
    private static final DigitalInput shooterPhotoelectric = new DigitalInput(ConveyorConstants.shooterPhotoelectric); //sensor closest to shooter
    public static Conveyor getInstance() {
        if (instance == null) instance = new Conveyor();
        return instance;
    }

    private Conveyor() {
        conveyorMotor = Util.createSparkMAX(ConveyorConstants.motor, MotorType.kBrushless);
        conveyorMotor.setInverted(true);
        conveyorMotor.burnFlash();
        storageMotor = Util.createSparkMAX(ConveyorConstants.motor, MotorType.kBrushless);
        storageMotor.setInverted(true);
        storageMotor.burnFlash();
        register();
    }

    public void setOpenLoop(double value) {
        conveyorMotor.set(value);
    }

    public void setStorageMotor(double value) {
        storageMotor.set(value);
    }

    public void stopConveyor() {
        conveyorMotor.stopMotor();
    }

    public void stopStorage() {
        storageMotor.stopMotor();
    }

    public void stop() {
        stopConveyor();
        stopStorage();
    }

    public boolean getIntakeSensor() {
        if(!ConveyorConstants.useColorSensor) {
            return !intakePhotoelectric.get();
        } else {
            return (RobotContainer.colorSensorV3.getProximity() >= ConveyorConstants.minimumProximity);
        }
    }

    public boolean getShooterSensor() {
        return !shooterPhotoelectric.get();
    }
}