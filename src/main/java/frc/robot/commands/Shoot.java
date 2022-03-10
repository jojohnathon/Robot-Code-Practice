package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class Shoot implements Command {
    private Shooter shooter = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Subsystem[] requirements = {shooter, intake};
    private double speed;
    Timer shootTimer;

    public Shoot(double speed) {
        this.speed = speed;
        shootTimer = new Timer();
    }

    @Override
    public void initialize() {
        shootTimer.start();
        shootTimer.reset();
    }

    @Override
    public void execute() {
        double nextSpeed = Shooter.FEEDFORWARD.calculate(speed);
        Shooter.PID_CONTROLLER.setSetpoint(speed);
        nextSpeed += Shooter.PID_CONTROLLER.calculate(Shooter.getShooterVelocity());
        nextSpeed /= Constants.kMaxVoltage;
        shooter.setOpenLoop(nextSpeed);

        if(Shooter.PID_CONTROLLER.atSetpoint() && shootTimer.hasElapsed(1.5)) { //TODO: Replace with Timeframe implementation
            intake.setConveyor(0.3); //Feed any balls into shooter once it has reached the desired angular velocity
            shooter.setStagingMotor(0.3);
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
