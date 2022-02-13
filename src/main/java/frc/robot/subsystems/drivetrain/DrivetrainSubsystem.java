package frc.robot.subsystems.drivetrain;

import java.util.Map;
import java.util.logging.Logger;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
// WPILib imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Package imports
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.drivetrain.commands.DriveCommand;
import frc.robot.subsystems.drivetrain.modules.CANSparkMaxModule;
import frc.robot.subsystems.drivetrain.modules.SwerveModule;
import frc.robot.subsystems.telemetry.Pigeon;
import frc.robot.subsystems.SubsystemFactory;

public abstract class DrivetrainSubsystem extends SubsystemBase {

    // Declaring Swerve Modules
    public SwerveModule frontLeftModule;
    public SwerveModule frontRightModule;
    public SwerveModule backLeftModule;
    public SwerveModule backRightModule;

    // Distance from center of wheel to center of wheel across the side of the bot in meters
    public static final double WHEEL_BASE = 0.4445;
    // Distance from center of wheel to center of wheel across the front of the bot in meters
    public static final double TRACK_WIDTH = 0.4445;

    private DriveCommand driveCommand;

    // Used to convert from ChassisSpeeds to SwerveModuleStates
    private SwerveDriveKinematics kinematics;

    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    private Logger logger = Logger.getLogger("DrivetrainSubsystem");

    // PID controller to prevent unintentional rotation when there is no rotation input. Takes in degrees nd outputs percent output.
    private PIDController anglePid = new PIDController(0.004, 0, 0);

    // To be used as the setpoint for the angle PID controller. This is in degrees.
    private double storedRotation;

    private boolean isFieldOriented = true;

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), kinematics,
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
        new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
    );

    /**
     * Initialize the drivetrain subsystem
     */
    public void init(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception {
        initializeSwerveModules(portAssignments, wheelOffsets);

        anglePid.enableContinuousInput(0.0, 360);

        // Create a new drive command to be reused later
        driveCommand = new DriveCommand(this);

        // Pass in the coordinates of each wheel relative to the center of the bot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // FL
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // FR
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // BL
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2) // BR
        );

        // Add the encoder readings to shuffleboard
        tab.addNumber("FL angle", () -> frontLeftModule.getAngle().getDegrees());
        tab.addNumber("FR angle", () -> frontRightModule.getAngle().getDegrees());
        tab.addNumber("BL angle", () -> backLeftModule.getAngle().getDegrees());
        tab.addNumber("BR angle", () -> backRightModule.getAngle().getDegrees());
    }

    public abstract void initializeSwerveModules(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception;

    @Override
    public void periodic() {
        // Run the drive command periodically
        if(driveCommand != null) {
            driveCommand.schedule();
        }
    }

    /** 
     * Drive the robot with percent output given a ChassisSpeeds object
     * <p>
     * This should be called periodically even if the input is not changing.
     * 
     * @param speeds Chasis speeds with vx and vy and omega between -1 and 1
    */
    public void drive(ChassisSpeeds speeds) {
        
        Pigeon gyro = SubsystemFactory.getInstance().getTelemetry().getPigeon();
        if(isFieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond, 
                gyro.getRotation2d()
            );
        }

        if(speeds.omegaRadiansPerSecond == 0) {
            // Dont move if we have no input.
            if(speeds.vxMetersPerSecond != 0 || speeds.vyMetersPerSecond != 0) {
                speeds.omegaRadiansPerSecond = MathUtil.clamp(anglePid.calculate(gyro.getAngle(), storedRotation), -1, 1);
            }
        } else {
            storedRotation = gyro.getAngle();
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1); // Normalize wheel speeds so we don't go faster than 100%

        poseEstimator.update(gyro.getRotation2d(), states);

        // Update SwerveModule states
        frontLeftModule.updateState(states[0]);
        frontRightModule.updateState(states[1]);
        backLeftModule.updateState(states[2]);
        backRightModule.updateState(states[3]);
    }

    /**
     * Reset the stored angle of the anglePid
     */
    public void resetAngleController() {
        storedRotation = 0.0;
        anglePid.reset();
    }

    /**
     * Turn field oriented on or off
     * 
     * @param val true to turn field oriented on, false to turn it off.
     */
    public void setFieldOriented(boolean val) {
        isFieldOriented = val;
    }

    /**
     * Determine if the bot is in field oriented drive mode
     * 
     * @return Returns true if in field oriented, otherwise, false.
     */
    public boolean getFieldOriented() {
        return isFieldOriented;
    }
}
