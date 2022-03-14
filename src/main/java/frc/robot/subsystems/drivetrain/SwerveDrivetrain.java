package frc.robot.subsystems.drivetrain;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.logging.Logger;

// WPILib imports
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
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

    public static final double MAX_LINEAR_SPEED = 2.74; // Meters per second
    public static final double MAX_LINEAR_ACCELERATION = 1; // Meters per second squared
    public static final double MAX_ROTATION_SPEED = 15.1; // Radians per second
    public static final double MAX_ROTATION_ACCELERATION = Math.PI; // Radians per second squared

    public PIDController xController;
    public PIDController yController;
    public ProfiledPIDController thetaController;

    // Command that calls drive method with user input.
    private DriveCommand driveCommand;

    // Used to convert from ChassisSpeeds to SwerveModuleStates
    private SwerveDriveKinematics kinematics;

    private Logger logger = Logger.getLogger("SwerveDrivetrain");
    
    // Odometry
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();
    
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    private NetworkTableEntry fieldOrientedToggle = tab.add("Field Oriented", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    private PIDController anglePid = new PIDController(.2, 0, 0);
    private double targetAngle = Double.NaN;
    private Instant startTime;

    /**
     * Initialize the drivetrain subsystem
     */
    public void init(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception {
        startTime = Instant.now();
        initializeSwerveModules(portAssignments, wheelOffsets);

        // Create a new drive command to be reused later
        driveCommand = new DriveCommand(this);

        // Pass in the coordinates of each wheel relative to the center of the bot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // FL
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // FR
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // BL
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2) // BR
        );

        poseEstimator = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), kinematics,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );

        anglePid.enableContinuousInput(0, 360);
        //anglePid.setTolerance(0.1);

        // Add the encoder readings to shuffleboard
        tab.addNumber("FL angle", () -> frontLeftModule.getAngle().getDegrees());
        tab.addNumber("FR angle", () -> frontRightModule.getAngle().getDegrees());
        tab.addNumber("BL angle", () -> backLeftModule.getAngle().getDegrees());
        tab.addNumber("BR angle", () -> backRightModule.getAngle().getDegrees());
        tab.addNumber("FL speed", frontLeftModule::getVelocity);
        tab.addNumber("FR speed", frontRightModule::getVelocity);
        tab.addNumber("BL speed", backLeftModule::getVelocity);
        tab.addNumber("BR speed", backRightModule::getVelocity);
        SmartDashboard.putData(field);
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            new ArrayList<Translation2d>(Arrays.asList(
                new Translation2d(2,0),
                new Translation2d(2,1),
                new Translation2d(4,1),
                new Translation2d(4,-1),
                new Translation2d(6,-1),
                new Translation2d(6,0)
            )),
            new Pose2d(7, 0, new Rotation2d()),
            new TrajectoryConfig(SwerveDrivetrain.MAX_LINEAR_SPEED, SwerveDrivetrain.MAX_LINEAR_ACCELERATION)
        );
        setDefaultCommand(new DriveCommand(this));

        field.getObject("traj").setTrajectory(trajectory);
    }
    public SwerveDrivePoseEstimator getposeEstimator(){
        return poseEstimator;
    }

    /**
     * Do not call this from outside of drivetrain!
     * <p>
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
        
        Pigeon gyro = (Pigeon) SubsystemFactory.getInstance().getTelemetry().getGyro();
        if(fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond, 
                gyro.getRotation2d()
            );
        }

        if(targetAngle != Double.NaN) {
            speeds.omegaRadiansPerSecond = anglePid.calculate(gyro.getAngle(), targetAngle);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_LINEAR_SPEED); // Normalize wheel speeds so we don't go faster than 100%

        // Update SwerveModule states
        frontLeftModule.updateState(SwerveModuleState.optimize(states[0], frontLeftModule.getAngle()));
        frontRightModule.updateState(SwerveModuleState.optimize(states[1], frontRightModule.getAngle()));
        backLeftModule.updateState(SwerveModuleState.optimize(states[2], backLeftModule.getAngle()));
        backRightModule.updateState(SwerveModuleState.optimize(states[3], backRightModule.getAngle()));
        
        field.setRobotPose(poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY(), gyro.getRotation2d());
        updateOdometry();
    }
    
    public void updateOdometry() {
        Gyro gyro = SubsystemFactory.getInstance().getTelemetry().getGyro();

        SwerveModuleState[] states = {
            new SwerveModuleState(frontLeftModule.getVelocity(), frontLeftModule.getAngle()),
            new SwerveModuleState(frontRightModule.getVelocity(), frontRightModule.getAngle()),
            new SwerveModuleState(backLeftModule.getVelocity(), backLeftModule.getAngle()),
            new SwerveModuleState(backRightModule.getVelocity(), backRightModule.getAngle())
        };

        poseEstimator.updateWithTime(Duration.between(startTime, Instant.now()).toMillis() / 1000.0, gyro.getRotation2d(), states);
        
    }

    /**
     * Stop all of the drive and angle motors.
     * 
     */
    public void stop() {
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
    }

    /**
     * Remove the target angle so that the bot can freely rotate.
     */
    public void removeTargetAngle() {
        targetAngle = Double.NaN;
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
        return poseEstimator.getEstimatedPosition();
    }

    public void resetLocation(Pose2d botLocation) {
        poseEstimator.resetPosition(botLocation, botLocation.getRotation());
        Pigeon pigeon = (Pigeon) SubsystemFactory.getInstance().getTelemetry().getGyro();
        pigeon.reset(botLocation.getRotation());
    }
}
