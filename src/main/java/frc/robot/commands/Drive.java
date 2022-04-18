package frc.robot.commands;
import frc.robot.*;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Drive implements Command {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private Subsystem[] requirements = {drivetrain};

    public Drive(State state) {
        this.state = state;
    }

    public enum State {
        TankDrive
    }

    private State state;

    @Override
    public void execute() {
       double leftThrottle = RobotContainer.getThrottle();


        double rightThrottle = RobotContainer.getAltThrottle();
        double left, right;

        switch(state) {
            case TankDrive:
                left = leftThrottle;
                right = rightThrottle;
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setOpenLoop(0, 0);
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
