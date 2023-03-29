package frc.robot.subsystems;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Units;
import frc.robot.Util;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain implements Subsystem {
    private static Drivetrain instance = null;
    private static final TalonFX
        leftMaster = Util.createTalonFX(DrivetrainConstants.leftMaster),
        leftSlave = Util.createTalonFX(DrivetrainConstants.leftSlave),
        rightMaster = Util.createTalonFX(DrivetrainConstants.rightMaster),
        rightSlave = Util.createTalonFX(DrivetrainConstants.rightSlave);
    
    public static final List<TalonFX> motors = List.of(leftMaster, leftSlave, rightMaster , rightSlave);


    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);
    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);
    public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(DrivetrainConstants.kMaxSpeedMPS, DrivetrainConstants.kMaxAcceleration);
    public static final ProfiledPIDController LEFT_PID_CONTROLLER = new ProfiledPIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD, constraints);
    public static final ProfiledPIDController RIGHT_PID_CONTROLLER = new ProfiledPIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD, constraints);
    public static DifferentialDriveOdometry ODOMETRY = new DifferentialDriveOdometry(Rotation2d.fromDegrees(RobotContainer.navX.getAngle()), getLeftEnc(), getRightEnc());
    
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(); //onbaord rio gyro
    //Simulation objects
    public DifferentialDrivetrainSim m_DrivetrainSim;
    private TalonFXSimCollection m_leftEncoderSim;
    private TalonFXSimCollection m_rightEncoderSim;
    //Field2d class shows the field on the sim GUI
    private Field2d m_fieldSim;
    private ADXRS450_GyroSim m_gyroSim;
    private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
    private final double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
    private final double kWheelRadiusInches = 3;
    private final int k100msPerSecond = 10;


    private Drivetrain() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        //leftSlave.setNeutralMode(NeutralMode.Coast);
        //rightSlave.setNeutralMode(NeutralMode.Coast);
        // Inverting opposite sides of the drivetrain
        List.of(leftMaster , leftSlave).forEach(motor -> motor.setInverted(false));
        List.of(rightMaster , rightSlave).forEach(motor -> motor.setInverted(true));
        

        if (RobotBase.isSimulation()) {
            m_DrivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
                KitbotMotor.kDoubleFalcon500PerSide, 
                KitbotGearing.k5p95, //TODO find actual gear ratio
                KitbotWheelSize.kSixInch, 
                /*
                * The standard deviations for measurement noise:
                * x and y:          0.001 m
                * heading:          0.001 rad
                * l and r velocity: 0.1   m/s
                * l and r position: 0.005 m
                */
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)); //not accurate at all probably
            m_leftEncoderSim = leftMaster.getSimCollection();
            m_rightEncoderSim = rightMaster.getSimCollection();
            m_gyroSim = new ADXRS450_GyroSim(m_gyro);

            m_fieldSim = new Field2d();
            SmartDashboard.putData("Field", m_fieldSim);
        }

        register();
    }

    @Override
    public void periodic() {
        ODOMETRY.update(Rotation2d.fromDegrees(-RobotContainer.navX.getAngle()),
        getLeftEncMeters(),
        getRightEncMeters());
        SmartDashboard.putNumber("Left Master output: ", getLeftEncVelocityMeters());
        //SmartDashboard.putNumber("Left Slave output: ", leftSlave.getMotorOutputPercent());
        SmartDashboard.putNumber("Right Master output: ", getRightEncVelocityMeters());
        SmartDashboard.putNumber("NavX heading", RobotContainer.navX.getAngle());
        //SmartDashboard.putNumber("Right Slave output: ", rightSlave.getMotorOutputPercent());
        

        m_fieldSim.setRobotPose(ODOMETRY.getPoseMeters());

    }
    @Override
    public void simulationPeriodic() {
        /* Pass the robot battery voltage to the simulated Talon SRXs */
        m_leftEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
        m_rightEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());

        /*
        * CTRE simulation is low-level, so SimCollection inputs
        * and outputs are not affected by SetInverted(). Only
        * the regular user-level API calls are affected.
        *
        * WPILib expects +V to be forward.
        * Positive motor output lead voltage is ccw. We observe
        * on our physical robot that this is reverse for the
        * right motor, so negate it.
        *
        * We are hard-coding the negation of the values instead of
        * using getInverted() so we can catch a possible bug in the
        * robot code where the wrong value is passed to setInverted().
        */
        m_DrivetrainSim.setInputs(
            m_leftEncoderSim.getMotorOutputLeadVoltage(),
            -m_rightEncoderSim.getMotorOutputLeadVoltage());

        /*
        * Advance the model by 20 ms. Note that if you are running this
        * subsystem in a separate thread or have changed the nominal
        * timestep of TimedRobot, this value needs to match it.
        */
        m_DrivetrainSim.update(0.02);

        /*
        * Update all of our sensors.
        *
        * Since WPILib's simulation class is assuming +V is forward,
        * but -V is forward for the right motor, we need to negate the
        * position reported by the simulation class. Basically, we
        * negated the input, so we need to negate the output.
        *
        * We also observe on our physical robot that a positive voltage
        * across the output leads results in a positive sensor velocity
        * for both the left and right motors, so we do not need to negate
        * the output any further.
        * If we had observed that a positive voltage results in a negative
        * sensor velocity, we would need to negate the output once more.
        */
        m_leftEncoderSim.setIntegratedSensorRawPosition(
                        distanceToNativeUnits(
                            m_DrivetrainSim.getLeftPositionMeters()
                        ));
        m_leftEncoderSim.setIntegratedSensorVelocity(
                        velocityToNativeUnits(
                            m_DrivetrainSim.getLeftVelocityMetersPerSecond()
                        ));
        m_rightEncoderSim.setIntegratedSensorRawPosition(
                        distanceToNativeUnits(
                            -m_DrivetrainSim.getRightPositionMeters()
                        ));
        m_rightEncoderSim.setIntegratedSensorVelocity(
                        velocityToNativeUnits(
                            -m_DrivetrainSim.getRightVelocityMetersPerSecond()
                        ));
        m_gyroSim.setAngle(-m_DrivetrainSim.getHeading().getDegrees());
    }

    private int distanceToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/(2 * Math.PI * Units.InchesToMeters(kWheelRadiusInches));
        double motorRotations = wheelRotations * kSensorGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }

    private int velocityToNativeUnits(double velocityMetersPerSecond){
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.InchesToMeters(kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
    }


  
    private static int kInverted = 1; //1 or -1
    public static int getkInvert() { //only for teleop driving, up to user to read this flag
        return kInverted;
    }

    public static void setOpenLoop(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    public static void setVoltages(double leftv, double rightv) {
        setOpenLoop(leftv/Constants.kMaxVoltage, rightv/Constants.kMaxVoltage);
    }

    public static void setInverted(boolean status) { //For defense, the back of the robot becomes the front
        if(status) {
            kInverted = -1;
        } else {
            kInverted = 1;
        }
    }

    public void stop() {
        setOpenLoop(0, 0);
    }

    /**
     * Zeroes encoders
     */
    public void resetEncoders() {
        resetEncoders(0, 0);
    }
    
    /**
     * Sets encoders to a specific value
     * @param left  left wheel value
     * @param right right wheel value
     */
    public void resetEncoders(int left, int right) {
        rightMaster.setSelectedSensorPosition(right);
        leftMaster.setSelectedSensorPosition(left);
    }

    /**
     * @return the left and right drivetrain velocities (in meters/sec) as a DifferentialDriveWheelSpeeds object
     */
    public static DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncVelocityMeters(), getRightEncVelocityMeters());
    }
    
    /**
     * @return the current position measurement of the left drivetrain encoder in talon native units (ticks)
     */
    public static double getLeftEnc() {
        return leftMaster.getSelectedSensorPosition();
    }
    
    /**
     * @return the current position measurement of the right drivetrain encoder in talon native units (ticks/)
     */
    public static double getRightEnc() {
        return rightMaster.getSelectedSensorPosition();
    }

    /**
     * @return the current position measurement of the left drivetrain encoder in meters
     */
    public static double getLeftEncMeters() {
        return Units.DrivetrainUnits.TicksToMeters(getLeftEnc());
    }
    
    /**
     * @return the current position measurement of the right drivetrain encoder in meters
     */
    public static double getRightEncMeters() {
        return Units.DrivetrainUnits.TicksToMeters(getRightEnc());
    }


    
    /**
     * @return the current velocity measurement of the left drivetrain encoder in talon native units (ticks/ds)
     */
    public static double getLeftEncVelocity() {
        return leftMaster.getSelectedSensorVelocity();
    }
    
    /**
     * @return the current velocity measurement of the right drivetrain encoder in talon native units (ticks/ds)
     */
    public static double getRightEncVelocity() {
        return rightMaster.getSelectedSensorVelocity();
    }

    /**
     * @return the current velocity measurement of the left drivetrain encoder in meters
     */
    public static double getLeftEncVelocityMeters() {
        return Units.DrivetrainUnits.TicksPerDecisecondToMPS(getLeftEncVelocity());
    }

    /**
     * @return the current velocity measurement of the right drivetrain encoder in meters
     */
    public static double getRightEncVelocityMeters() {
        return Units.DrivetrainUnits.TicksPerDecisecondToMPS(getRightEncVelocity());
    }
    
    /* Static class to contain the speeds of each side of the drivetrain */
    public static class WheelState {
        public double left, right;
        
        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public static Drivetrain getInstance() {
        if(instance == null) instance = new Drivetrain();
        return instance;
    }
}