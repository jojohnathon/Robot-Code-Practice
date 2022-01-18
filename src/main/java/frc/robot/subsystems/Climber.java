package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climber implements Subsystem {
    private static Climber instance = null;
    private Climber() {
        register();
    }
    public static Climber getInstance() {
        if(instance == null) instance = new Climber();
        return instance;
    }
}
