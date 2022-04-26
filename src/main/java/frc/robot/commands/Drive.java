package frc.robot.commands;
import frc.robot.*;
import frc.robot.Constants.DriverConstants;
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
        TankDrive, CurvatureDrive
    }

    private State state;

    @Override
    public void execute() {
        // double leftThrottle = RobotContainer.getThrottle();
        double altThrottle = RobotContainer.getAltThrottle();
        double left, right;
        double throttle = RobotContainer.getThrottle();
        double turn = RobotContainer.getTurn();

        switch(state) {
            case TankDrive:
                left = throttle * DriverConstants.kDriveSens;
                right = altThrottle * DriverConstants.kDriveSens;
                break;
            case CurvatureDrive:
                if (throttle != 0) {
                    left = (throttle + throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
                    right = (throttle - throttle * turn *DriverConstants.kTurnSens) * DriverConstants.kDriveSens;

                    double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));

                    if(maxMagnitude > DriverConstants.kDriveSens) {
                        left = left / maxMagnitude * DriverConstants.kDriveSens;
                        right = right / maxMagnitude * DriverConstants.kDriveSens;
                    } else {
                        left = turn * DriverConstants.kTurnInPlaceSens;
                        right = -turn * DriverConstants.kTurnInPlaceSens;
                    }
                    
                    break;
                }
            default:
                left = right = 0;
        } 
        Drivetrain.setOpenLoop(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setOpenLoop(0, 0);
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
