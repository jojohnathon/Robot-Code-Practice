package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter implements Subsystem {
    private static final CANSparkMax master = new CANSparkMax(ShooterConstants.master, MotorType.kBrushless);
    private static final CANSparkMax slave = new CANSparkMax(ShooterConstants.slave, MotorType.kBrushless);

    private static Shooter instance = null;
    private Shooter() {
        slave.follow(master);
    }

    public static Shooter getInstance() {
        if(instance == null) instance = new Shooter();
        return instance;
    }
}
