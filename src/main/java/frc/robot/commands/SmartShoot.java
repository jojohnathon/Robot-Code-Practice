package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionMount;

public class SmartShoot extends Shoot {
    private Subsystem[] extraReqs = {VisionMount.getInstance()};
    public SmartShoot(double distance) {
        super(distanceToVelocity(distance));
        VisionMount.getInstance().setAngle(VisionConstants.mountAngle);
    }

    public static double distanceToVelocity(double distance) {
        //TODO: Do math
        return 0.1 * distance;
    }


    public Set<Subsystem> getRequirements() {
        Set<Subsystem> requirements = Set.of(extraReqs);
        requirements.addAll(super.getRequirements());
        return Set.of(extraReqs);
    }
}
