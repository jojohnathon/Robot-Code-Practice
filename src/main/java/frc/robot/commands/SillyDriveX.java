package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class SillyDriveX implements Command {

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return Set.of(Drivetrain.getInstance());
    }
    private double meters;

    public SillyDriveX(double meters) {
        this.meters = meters;
    }
    
    public void initialize() {
        Drivetrain.getInstance().resetEncoders();
    }
    public void execute() {
        Drivetrain.setOpenLoop(DrivetrainConstants.sdx, DrivetrainConstants.sdx);
    }

    public boolean isFinished() {
        double avgdist = (Drivetrain.getLeftEncMeters() + Drivetrain.getRightEncMeters()) / 2;
        return (avgdist > this.meters);
    }
}
