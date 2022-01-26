package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class AllianceIntake implements Command {
    private Subsystem[] requirements = {RobotContainer.intake};
    public enum AllianceColors {
        red, blue;
    }
    private AllianceColors allianceColor;
    private RobotContainer robot;
    public AllianceIntake(AllianceColors allianceColor) {
        this.allianceColor = allianceColor;
        robot = RobotContainer.getInstance();
    }

    public void execute() {
        switch(allianceColor) {
            case red:
                if(robot.getColor().getR() > 200 && robot.getProximity() > 1680) { //TODO: check that the object is adequately red
                    CommandScheduler.getInstance().schedule(); //TODO: schedule Intake of ball
                }
            case blue:
                if(robot.getColor().getB() > 200 && robot.getProximity() > 1680) { //TODO: check that the object is adequately blue                    
                    CommandScheduler.getInstance().schedule(); //TODO: schedule Intake of ball
                }
        }
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
