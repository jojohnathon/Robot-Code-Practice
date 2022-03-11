package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.CustomUtil.Timeframe;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class Shoot implements Command {
    private Shooter shooter = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Subsystem[] requirements = {shooter, intake};
    private double speed;
    private Timeframe<Integer> timeframe;

    public Shoot(double speed) {
        this.speed = speed;
        timeframe = new Timeframe<>(1.5, 1.0/Constants.dt);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double nextSpeed = Shooter.FEEDFORWARD.calculate(speed);
        Shooter.PID_CONTROLLER.setSetpoint(speed);
        nextSpeed += Shooter.PID_CONTROLLER.calculate(Shooter.getShooterVelocity());
        nextSpeed /= Constants.kMaxVoltage;
        shooter.setOpenLoop(nextSpeed);
        if(Shooter.PID_CONTROLLER.atSetpoint()) {
            timeframe.update(1);
        } else {
            timeframe.update(0);
        }

        if(timeframe.percentEqual(1) >= 0.85) { //TODO: Replace with Timeframe implementation
            intake.setConveyor(0.3); //Feed any balls into shooter once it has reached the desired angular velocity
            shooter.setStagingMotor(0.3);
        } else {
            intake.setConveyor(0);
            shooter.setStagingMotor(0);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        intake.stopIntake();
    }
    
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
