package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SillyShoot implements Command {
    private Subsystem[] requirements = {Shooter.getInstance(), Intake.getInstance()};
    
    Timer encoderFallback;
    
    public SillyShoot() {
        encoderFallback = new Timer();
        
    }

    @Override
    public void initialize() {
        encoderFallback.start();
        encoderFallback.reset();
        
    }
    @Override
    public void execute(){

        Shooter.getInstance().setOpenLoop(0.65);

        if(Shooter.getShooterVelocity() > 3000){ //TODO: encoder tuning
            //Intake.getInstance().intake(0.55);
            Intake.getInstance().setConveyor(0.50);     
            Shooter.getInstance().setStagingMotor(0.5);
            encoderFallback.reset();
        } else if(encoderFallback.hasElapsed(1.5)){
            //Intake.getInstance().intake(0.55);
            Intake.getInstance().setConveyor(0.50); 
            
            Shooter.getInstance().setStagingMotor(0.3);
            
        }
        
        
    }
    
    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stop();
        Intake.getInstance().stopIntake();
        Intake.getInstance().stopConveyor();
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
