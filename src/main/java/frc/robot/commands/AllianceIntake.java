package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AllianceIntake implements Command {
    private Intake intake = Intake.getInstance();
    private Arm arm = Arm.getInstance();
    private Subsystem[] requirements = {};
    public enum AllianceColors {
        red, blue;
    }
    private AllianceColors allianceColor;
    private RobotContainer robot;
    public AllianceIntake(AllianceColors allianceColor) {
        this.allianceColor = allianceColor;
    }
/*
    public void execute() {
        final Command intakeCommand = new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                            new RunCommand( ()->arm.rotate(-0.4), arm),
                                            new RunCommand( ()->intake.intake(0.5), intake), 
                                            new RunCommand( ()->intake.setConveyor(0.5), intake)), 
                                        new WaitCommand(1.7), 
                                        new ParallelCommandGroup(
                                            new RunCommand( ()->arm.rotate(0.35), arm),
                                            new InstantCommand(intake::stopIntake, intake)));
        switch(allianceColor) {
            case red:
                if(RobotContainer.getColor().red > VisionConstants.minimumSimilarity && robot.getProximity() > VisionConstants.minimumProximity) { //TODO: check that the object is adequately red
                    CommandScheduler.getInstance().schedule(intakeCommand); 
                }
            case blue:
                if(RobotContainer.getColor().blue > VisionConstants.minimumSimilarity && robot.getProximity() > VisionConstants.minimumProximity) { //TODO: check that the object is adequately blue                    
                    CommandScheduler.getInstance().schedule(intakeCommand); 
                }
        }
    }
*/
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
