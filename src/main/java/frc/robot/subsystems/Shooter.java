package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util;
import frc.robot.Constants.ShooterConstants;

public class Shooter implements Subsystem {
    private static final CANSparkMax master = Util.createSparkMAX(ShooterConstants.master, MotorType.kBrushless);
    private static final CANSparkMax slave = Util.createSparkMAX(ShooterConstants.slave, MotorType.kBrushless);
    public static PIDController PID_CONTROLLER;
    public static SimpleMotorFeedforward FEEDFORWARD;
    private static RelativeEncoder encoder = master.getEncoder();

    private static Shooter instance = null;

    private Shooter() {
        PID_CONTROLLER = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        FEEDFORWARD = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
        slave.follow(master);
    }

    public static Shooter getInstance() {
        if(instance == null) instance = new Shooter();
        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter flywheel speed: ", getShooterVelocity());
    }

    public void setOpenLoop(final double value){
        master.set(value);
    }

    public void stop(){
        master.stopMotor();
    }

    public static double getShooterVelocity() {
        return encoder.getVelocity();
    }
}
