package frc.robot.subsystems.drivetrain;

import java.util.Map;
import java.util.logging.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.commands.DriveCommand;
import frc.robot.subsystems.drivetrain.modules.SwerveModule;
import frc.robot.subsystems.telemetry.Pigeon;

/**
 * A SwerveDrivetrain subsystem represents a drivetrain with four swerve modules.
 */
public abstract class SwerveDrivetrain extends SubsystemBase {

    // Declaring Swerve Modules
    public SwerveModule frontLeftModule;
    public SwerveModule frontRightModule;
    public SwerveModule backLeftModule;
    public SwerveModule backRightModule;

    // Distance from center of wheel to center of wheel across the side of the bot in meters
    public static final double WHEEL_BASE = 0.4445;
    // Distance from center of wheel to center of wheel across the front of the bot in meters
    public static final double TRACK_WIDTH = 0.4445;

    public static final double MAX_LINEAR_SPEED = 3.5; // Meters per second
    public static final double MAX_LINEAR_ACCELERATION = 3.5; // Meters per second squared
    public static final double MAX_ROTATION_SPEED = 15.1; // Radians per second
    public static final double MAX_ROTATION_ACCELERATION = Math.PI; // Radians per second squared

    // In meters per second
    public static final double IS_MOVING_TRANSLATION_TOLERANCE = 3.5;
    // In radians per second
    public static final double IS_MOVING_ROTATION_TOLERANCE = 1;

    // PID controllers to correct error while following autonomous paths
    public PIDController xController;
    public PIDController yController;
    public PIDController thetaController;


    // Used to convert from ChassisSpeeds to SwerveModuleStates
    private SwerveDriveKinematics kinematics;

    private Logger logger = Logger.getLogger("DrivetrainSubsystem");
    
    // Odometry
    private SwerveDriveOdometry odometry;
    // Used for displaying the estimated position of the robot on the field.
    private Field2d field = new Field2d();
    
    // Output and input
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    private NetworkTableEntry fieldOrientedToggle = tab.add("Field Oriented", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    // PID controller for snapping to angles
    private PIDController anglePid = new PIDController(.12, 0, 0);

    // Angle snapping logic
    private double targetAngle = Double.NaN;
    public boolean isAtTargetAngle = false;

    private boolean isInBrakeMode = false;

    /**
     * Initialize the drivetrain subsystem
     */
    public void init(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception {
        initializeSwerveModules(portAssignments, wheelOffsets);

        // Pass in the coordinates of each wheel relative to the center of the bot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // FL
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // FR
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // BL
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // BR
        );

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());

        anglePid.enableContinuousInput(0, 360);
        anglePid.setTolerance(3);

        setDefaultCommand(new DriveCommand(this));

        // Add the encoder readings to shuffleboard to be used for wheel calibration and debugging
        tab.addNumber("FL angle", () -> frontLeftModule.getAngle().getDegrees());
        tab.addNumber("FR angle", () -> frontRightModule.getAngle().getDegrees());
        tab.addNumber("BL angle", () -> backLeftModule.getAngle().getDegrees());
        tab.addNumber("BR angle", () -> backRightModule.getAngle().getDegrees());

        tab.add(field);
    }

    /**
     * Initialize the swerve modules
     * 
     * @param portAssignments The ports for the swerve modules
     * @param wheelOffsets The offsets for the modules
     * @throws Exception If there is an issue acquiring a port.
     */
    protected abstract void initializeSwerveModules(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception;

    /** 
     * Drive the robot with percent output given a ChassisSpeeds object
     * <p>
     * This should be called periodically even if the input is not changing.
     * 
     * @param speeds Chassis speeds with vx and vy <= max linear speed and omega < max rotation speed
    */
    public void drive(ChassisSpeeds speeds, boolean fieldOriented) {
        // Brake mode logic
        if(isInBrakeMode) {
            // Rotate all the wheels to point to the center of the bot so we are hard to move.
            frontLeftModule.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
            frontRightModule.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(145)));
            backLeftModule.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
            backRightModule.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
            return;
        }
        // Field oriented logic
        Gyro gyro = SubsystemFactory.getInstance().getTelemetry().getGyro();
        if(fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond, 
                Rotation2d.fromDegrees(gyro.getAngle())
            );
        }
        // Angle Snapping logic
        SmartDashboard.putNumber("Target angle: ", targetAngle);
        if(!Double.isNaN(targetAngle)) {
            speeds.omegaRadiansPerSecond = anglePid.calculate(gyro.getAngle());
            isAtTargetAngle = anglePid.atSetpoint();
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        // Normalize wheel speeds so we don't try to go faster than our max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_LINEAR_SPEED);
        
        odometry.update(gyro.getRotation2d(), getModuleStates());
        
        // Update the display of our estimated position
        field.setRobotPose(
            odometry.getPoseMeters().getX(),
            odometry.getPoseMeters().getY(),
            gyro.getRotation2d()
        );

        // Update SwerveModule states
        frontLeftModule.updateState(SwerveModuleState.optimize(states[0], frontLeftModule.getAngle()));
        frontRightModule.updateState(SwerveModuleState.optimize(states[1], frontRightModule.getAngle()));
        backLeftModule.updateState(SwerveModuleState.optimize(states[2], backLeftModule.getAngle()));
        backRightModule.updateState(SwerveModuleState.optimize(states[3], backRightModule.getAngle()));
    }

    /**
     * Return true if the robot is stopped or moving very slowly.
     * <p>
     * This is mainly for ignoring vision measurements while we are moving.
     * 
     * @return
     */
    public boolean isMoving() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        if(Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)) > IS_MOVING_TRANSLATION_TOLERANCE) {
            return true;
        } else if(Math.abs(speeds.omegaRadiansPerSecond) > IS_MOVING_ROTATION_TOLERANCE) {
            return true;
        }
        return false;
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    /**
     * Stop all of the swerve modules from moving
     */
    public void stop() {
        drive(new ChassisSpeeds(), false);
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    /**
     * Set the target angle for the bot to rotate to.
     * 
     * @param targetAngle The target angle.
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle.getDegrees();
        anglePid.setSetpoint(this.targetAngle);
    }

    /**
     * Remove the target angle so that the bot can freely rotate.
     */
    public void removeTargetAngle() {
        targetAngle = Double.NaN;
        anglePid.reset();
        isAtTargetAngle = false;
    }

    /**
     * Return true if a target angle has been set.
     * 
     * @return true if a target angle has been set.
     */
    public boolean hasTargetAngle() {
        return !Double.isNaN(targetAngle);
    }

    /**
     * Determine if the target angle has been reached
     * 
     * @return True if the bot is within tolerance of the target angle.
     */
    public boolean atTargetAngle() {
        return isAtTargetAngle;
    }

    /**
     * Turn field oriented on or off
     * 
     * @param val true to turn field oriented on, false to turn it off.
     */
    public void setFieldOriented(boolean val) {
        fieldOrientedToggle.setBoolean(val);
    }

    /**
     * Determine if the bot is in field oriented drive mode
     * 
     * @return Returns true if in field oriented, otherwise, false.
     */
    public boolean getFieldOriented() {
        return fieldOrientedToggle.getBoolean(true);
    }

    /**
     * Get the current estimated position of the robot in meters.
     * <p>
     * x+ is forwards, y+ is right.
     * 
     * @return The estimated position of the bot.
     */
    public Pose2d getLocation() {
        Pigeon pigeon = (Pigeon) SubsystemFactory.getInstance().getTelemetry().getGyro();
        return new Pose2d(odometry.getPoseMeters().getTranslation(), pigeon.getRotation2d());
    }

    /**
     * Reset the estimated position of the robot to a given Pose2d
     * 
     * @param botLocation the position to reset the estimated position to.
     */
    public void resetLocation(Pose2d botLocation) {
        Pigeon pigeon = (Pigeon) SubsystemFactory.getInstance().getTelemetry().getGyro();
        odometry.resetPosition(botLocation, botLocation.getRotation());
        pigeon.reset(botLocation.getRotation());
    }

    /**
     * Get the odometry object of the drivetrain
     * 
     * @return The odometry object of the drivetrain.
     */
    public SwerveDriveOdometry getSwerveDriveOdometry(){
        return odometry;
    }

    public void enableBrakeMode() {
        isInBrakeMode = true;
    }

    public void disableBrakeMode() {
        isInBrakeMode = false;
    }
}
