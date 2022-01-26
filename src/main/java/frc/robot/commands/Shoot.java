package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class Shoot implements Command {
    private Subsystem[] requirements = {RobotContainer.shooter};
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
        Shooter.getInstance().setOpenLoop(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stop();
    }
    
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
