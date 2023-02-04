package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class ActuateArm implements Command {
    private static Subsystem[] requirements = {Intake.getInstance()};
    private Timer timer;
    public ActuateArm() {
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(1.7)) {
            Intake.getInstance().setArm(0.0);
            timer.stop();
        } else {
            Intake.getInstance().setArm(0.0);
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void stop() {
        end(false);
    }

    public void end(boolean interrupted) {
        Intake.getInstance().stopArm();
        timer.stop();
        //TODO: check if this method works
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
    
}
