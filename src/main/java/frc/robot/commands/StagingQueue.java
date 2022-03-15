package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StagingQueue implements Command {
    private Shooter shooter = Shooter.getInstance();
    private Subsystem[] requirements = {shooter};
    
    @Override
    public void execute() {
        if(!shooter.getShooterSensor() && Intake.getInstance().getIntakeSensor()) { //If there is room in the staging area, and we have a ball in conveyor, then stage the cargo
            shooter.setStagingMotor(0.70);
        } else {
            shooter.setStagingMotor(0.0);
        }
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
