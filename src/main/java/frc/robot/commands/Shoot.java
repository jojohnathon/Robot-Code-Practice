package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public class Shoot implements Command {
    private Subsystem[] requirements = {RobotContainer.shooter};
    public Shoot() {

    }
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
