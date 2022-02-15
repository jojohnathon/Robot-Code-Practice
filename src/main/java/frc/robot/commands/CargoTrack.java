package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.LEDMode;
import frc.robot.RobotContainer.VisionPipeline;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

public class CargoTrack implements Command {

    private static final PIDController TURN_PID_CONTROLLER = new PIDController(VisionConstants.kPTurn,
            VisionConstants.kITurn, VisionConstants.kDTurn);
    private static final PIDController DIST_PID_CONTROLLER = new PIDController(VisionConstants.kPDist,
            VisionConstants.kIDist, VisionConstants.kDDist);

    private Subsystem[] requirements = { Drivetrain.getInstance() };

    public CargoTrack() {

    }

    @Override
    public void initialize() {
        RobotContainer.getInstance().setIntakeLEDMode(LEDMode.OFF);
        RobotContainer.getInstance().setIntakePipeline(RobotContainer.allianceToPipeline());

    }

    @Override
    public void execute() {

        double left, right;

        double turnError = RobotContainer.getIntakeXOffset();
        double distError = RobotContainer.getIntakeYOffset();

        if (turnError < VisionConstants.kTurnTolerance) turnError = 0;
        if (distError < VisionConstants.kDistTolerance) distError = 0;

        double throttle = DIST_PID_CONTROLLER.calculate(distError, 0);
        double turn = TURN_PID_CONTROLLER.calculate(turnError, 0);
        //double length, width, len, shrt;
        
        /*length = RobotContainer.getInstance().limelight.getEntry("thor").getDouble(0);
        width = RobotContainer.getInstance().limelight.getEntry("tvert").getDouble(0);
        len = RobotContainer.getInstance().limelight.getEntry("tlong").getDouble(0);
        shrt = RobotContainer.getInstance().limelight.getEntry("tshort").getDouble(0);*/



        if (throttle != 0) {
            throttle *= DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens;
            turn *= DrivetrainConstants.kMaxCurvature * DriverConstants.kTurnSens * throttle;

            DifferentialDriveWheelSpeeds wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
            wSpeeds.desaturate(DrivetrainConstants.kMaxSpeedMPS);

            left = Drivetrain.FEEDFORWARD.calculate(wSpeeds.leftMetersPerSecond) / Constants.kMaxVoltage;
            right = Drivetrain.FEEDFORWARD.calculate(wSpeeds.rightMetersPerSecond) / Constants.kMaxVoltage;

        } else {
            // Turns in place when there is no throttle input
            left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
            right = -turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;

            left = Drivetrain.FEEDFORWARD.calculate(left) / Constants.kMaxVoltage;
            right = Drivetrain.FEEDFORWARD.calculate(right) / Constants.kMaxVoltage;
        }

        //Drivetrain.setOpenLoop(left, right);

    }

    @Override
    public boolean isFinished() {
        //TODO: Add a suitable end condition
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getInstance().setIntakeLEDMode(LEDMode.OFF);
        Drivetrain.setOpenLoop(0.0, 0.0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}