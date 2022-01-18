package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    private static Intake instance = null;
    private Intake() {
        register();
    }

    public static Intake getInstance() {
        if(instance == null) instance = new Intake();
        return instance;
    }
}
