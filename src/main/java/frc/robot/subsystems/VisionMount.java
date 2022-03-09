package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.VisionConstants;

public class VisionMount implements Subsystem {
    private static VisionMount instance = null;
    private static Servo limelightServo;
    private VisionMount() {
        limelightServo = new Servo(VisionConstants.servoChannel);
        register();
    }

    public void setAngle(double degrees) {
        limelightServo.setAngle(degrees);
    }

    public void set(double amount) {
        limelightServo.set(amount);
    }
    
    public static VisionMount getInstance() {
        if(instance == null) instance = new VisionMount();
        return instance;
    }
}
