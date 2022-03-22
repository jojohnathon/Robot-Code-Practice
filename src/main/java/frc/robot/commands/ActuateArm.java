package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ActuateArm implements Command {
    private static Subsystem[] requirements = {Arm.getInstance()};
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
        
        if(timer.hasElapsed(1.7)) {
            Arm.getInstance().setOpenLoop(0.0);
            timer.stop(); 
        } else {
            Arm.getInstance().setOpenLoop(0.05);
        }
    }

    public boolean isFinished() {
        return false;
    }
    public void stop() {
        end(false);
    }

    public void end(boolean interrupted) {
        Arm.getInstance().stopArm();
        timer.stop();
        CommandScheduler.getInstance().schedule(new RunCommand(() -> Arm.getInstance().setOpenLoop(-0.05), Arm.getInstance()).withTimeout(timer.get())
            .andThen(new InstantCommand(() -> Arm.getInstance().stopArm()))); //retract arm TODO: test whether or not this can require Arm subsystem and work
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
