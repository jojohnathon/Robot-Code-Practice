package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ExampleSubsystem implements Subsystem {
    //Initialize motors here or in the constructor (you will always use some sort of motor controller, such as TalonFX for Falcons, or CANSparkMax for NEO motors)

    public ExampleSubsystem() {


        register(); //Necessary for the subsystem to work with the Command-Based framework
    }
}
