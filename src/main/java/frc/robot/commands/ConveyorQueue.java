package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;

public class ConveyorQueue implements Command {
    private Conveyor conveyor = Conveyor.getInstance();
    private Subsystem[] requirements = {conveyor};

    public ConveyorQueue() {

    }

    @Override
    public void execute() {
        if(!conveyor.getShooterSensor() && conveyor.getIntakeSensor()) { //Ball in front of conveyor and no ball in the back of the conveyor 
            conveyor.setOpenLoop(0.2);
        } else {
            conveyor.stop();
        }
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
