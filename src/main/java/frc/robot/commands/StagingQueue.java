package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;

public class StagingQueue implements Command {
    private Shooter shooter = Shooter.getInstance();
    private Subsystem[] requirements = {shooter};
    
    @Override
    public void execute() {
        if(shooter.getShooterSensor()) { //Ball in back of conveyor that needs to be propped into storage
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
