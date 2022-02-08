package frc.robot.commands;


import java.util.Set;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer.LEDMode;
import frc.robot.subsystems.Drivetrain;

public class VisionTrack implements Command {

    private static final PIDController TURN_PID_CONTROLLER = new PIDController(VisionConstants.kPTurn,
            VisionConstants.kITurn, VisionConstants.kDTurn);
    private static final PIDController DIST_PID_CONTROLLER = new PIDController(VisionConstants.kPDist,
            VisionConstants.kIDist, VisionConstants.kDDist);

    private Subsystem[] requirements = { RobotContainer.drivetrain };

    public VisionTrack() {

    }

    @Override
    public void initialize() {
        //TODO: Configure photonvision pipeline

    }

    @Override
    public void execute() {

        double left, right;

        double turnError = RobotContainer.getSnapshot().getBestTarget().getPitch();
        double distError = RobotContainer.getSnapshot().getBestTarget().getYaw();

        if (turnError < VisionConstants.kTurnTolerance) turnError = 0;
        if (distError < VisionConstants.kDistTolerance) distError = 0;

        double throttle = DIST_PID_CONTROLLER.calculate(distError, 0);
        double turn = TURN_PID_CONTROLLER.calculate(turnError, 0);

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

        Drivetrain.setOpenLoop(left, right);

    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.camera.setLED(VisionLEDMode.kOff);
        Drivetrain.setOpenLoop(0.0, 0.0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}