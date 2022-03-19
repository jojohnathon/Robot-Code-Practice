package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class SillyDriveX implements Command {

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(Drivetrain.getInstance());
    }
    private double meters;
    private boolean reverse;

    public SillyDriveX(double meters, boolean reverse) {
        this.meters = meters;
        this.reverse = reverse;
    }

    public SillyDriveX(double meters) {
        this.meters = meters;
        this.reverse = false;
    }
    
    public void initialize() {
        Drivetrain.getInstance().resetEncoders();
    }
    public void execute() {
        if (this.reverse) Drivetrain.setOpenLoop(-DrivetrainConstants.sdx, -DrivetrainConstants.sdx);
        else Drivetrain.setOpenLoop(-DrivetrainConstants.sdx, -DrivetrainConstants.sdx);
    }

    public boolean isFinished() {
        double avgdist = (Drivetrain.getLeftEncMeters() + Drivetrain.getRightEncMeters()) / 2;
        if (this.reverse) return (avgdist < -this.meters);
        else return (avgdist > this.meters);
    }

    public void end(boolean interrupted) {
        Drivetrain.getInstance().stop();
    }
}
