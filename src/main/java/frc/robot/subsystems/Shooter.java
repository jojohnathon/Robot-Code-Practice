package frc.robot.subsystems;

public class Shooter {
    private static Shooter instance = null;
    private Shooter() {
        
    }

    public static Shooter getInstance() {
        if(instance == null) instance = new Shooter();
        return instance;
    }
}
