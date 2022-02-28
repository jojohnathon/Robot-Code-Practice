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
        if(conveyor.getIntakeSensor()) { //Ball in front of conveyor 
            conveyor.setOpenLoop(0.85);
        } else {
            conveyor.stopConveyor();
        }
        if(conveyor.getShooterSensor()) { //Ball in back of conveyor that needs to be propped into storage
            conveyor.setStorageMotor(0.70);
        } else {
            conveyor.stopStorage();
        }
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
