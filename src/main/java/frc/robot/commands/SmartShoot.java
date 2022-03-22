package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.VisionConstants;

public class SmartShoot extends Shoot {
    private Subsystem[] extraReqs = {};
    public SmartShoot(double distance) {
        super(distanceToVelocity(distance));
    }

    public static double distanceToVelocity(double distance) {
        //TODO: Do math
        return 0.1 * distance;
    }


    public Set<Subsystem> getRequirements() {
        return super.getRequirements();
    }
}
