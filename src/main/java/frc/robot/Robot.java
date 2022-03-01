// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.RobotContainer;
import frc.robot.Autonomous.Auto;
import frc.robot.commands.CargoTrack;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private RobotContainer robot;
  private PowerDistribution pdp = new PowerDistribution();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    robot = RobotContainer.getInstance();
    m_chooser.setDefaultOption("Shoot First", RobotContainer.getAutonomousCommand(Auto.Selection.SHOOTFIRST));
    m_chooser.addOption("Intake First", RobotContainer.getAutonomousCommand(Auto.Selection.INTAKEFIRST));
    SmartDashboard.putData("Auto choices", m_chooser);
    Drivetrain.getInstance().resetEncoders();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("dt left enc", Drivetrain.getLeftEnc());
    SmartDashboard.putNumber("dt right enc", Drivetrain.getRightEnc());
    //double length, width, len, shrt;
    
    /*length = RobotContainer.limelight.getEntry("thor").getDouble(0);
    width = RobotContainer.getInstance().limelight.getEntry("tvert").getDouble(0);
    len = RobotContainer.getInstance().limelight.getEntry("tlong").getDouble(0);
    shrt = RobotContainer.getInstance().limelight.getEntry("tshort").getDouble(0);
    //SmartDashboard.putNumber("charles", length);
    SmartDashboard.putNumberArray("hor, vert, ratio", new double[] {length, width, length/width});
    SmartDashboard.putNumberArray("long, short, ratio", new double[] {len, shrt, length/shrt});*/

    /*SmartDashboard.putNumber("CV3 red", RobotContainer.getColor().red);
    SmartDashboard.putNumber("CV3 green", RobotContainer.getColor().green);
    SmartDashboard.putNumber("CV3 blue", RobotContainer.getColor().blue);*/

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  private Command auto;
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(auto = m_chooser.getSelected());
    pdp.clearStickyFaults();
    //CommandScheduler.getInstance().schedule(new VisionTrack());
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //
    /*switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }*/
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if(auto != null) auto.cancel();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
