package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveXMeters implements Command {
    private Drivetrain drivetrain = RobotContainer.drivetrain;
    private Subsystem[] requirements = {drivetrain};
    private TrapezoidProfile.State goal;
    private TrapezoidProfile profile;
    
    public DriveXMeters(TrapezoidProfile.State goal) {
        this.goal = goal;
        profile = new TrapezoidProfile(Drivetrain.constraints, this.goal);
    }

    public DriveXMeters(double distance, double maxSpeedMPS) {
        this(new TrapezoidProfile.State(distance, maxSpeedMPS));

    }


    @Override
    public void initialize() {
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State profileCalc = profile.calculate(Constants.dt);
        Drivetrain.LEFT_PID_CONTROLLER.calculate(Drivetrain.getLeftEnc(), profileCalc);
        Drivetrain.RIGHT_PID_CONTROLLER.calculate(Drivetrain.getRightEnc(), profileCalc);
        profile = new TrapezoidProfile(Drivetrain.constraints, goal, profileCalc);
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
