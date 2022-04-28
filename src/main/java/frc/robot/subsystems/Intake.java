package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake implements Subsystem{
    private static final CANSparkMax rollerMotor = Util.createSparkMAX(IntakeConstants.rollerMotor, MotorType.kBrushless);
    private static final CANSparkMax conveyorMotor = Util.createSparkMAX(ConveyorConstants.conveyorMotor, MotorType.kBrushless);
    
    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }

    private Intake(){
        conveyorMotor.setInverted(true);
        rollerMotor.setInverted(false);
        register();
    }

    public void intake(double value) {
        
    }
}
