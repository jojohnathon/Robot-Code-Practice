package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ConveyorQueue implements Command {
    private Intake intake = Intake.getInstance();
    private Subsystem[] requirements = {intake};

    public ConveyorQueue() {

    }

    @Override
    public void execute() {
        if(intake.getIntakeSensor()) { //Ball in front of conveyor 
            intake.setConveyor(0.85);
        } else {
            intake.stopConveyor();
        }
        
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
