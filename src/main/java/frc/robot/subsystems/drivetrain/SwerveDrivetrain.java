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

    public static final double MAX_LINEAR_SPEED = 3.5; // Meters per second
    public static final double MAX_LINEAR_ACCELERATION = 3; // Meters per second squared
    public static final double MAX_ROTATION_SPEED = 15.1; // Radians per second
    public static final double MAX_ROTATION_ACCELERATION = Math.PI; // Radians per second squared

    public PIDController xController;
    public PIDController yController;
    public PIDController thetaController;


    // Used to convert from ChassisSpeeds to SwerveModuleStates
    private SwerveDriveKinematics kinematics;

    private Logger logger = Logger.getLogger("DrivetrainSubsystem");
    
    // Odometry
    private SwerveDriveOdometry odometry;
    private Field2d field = new Field2d();
    
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    private NetworkTableEntry fieldOrientedToggle = tab.add("Field Oriented", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    private PIDController anglePid = new PIDController(.12, 0, 0);
    private double targetAngle = Double.NaN;

    /**
     * Initialize the drivetrain subsystem
     */
    public void init(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception {
        initializeSwerveModules(portAssignments, wheelOffsets);

        // Pass in the coordinates of each wheel relative to the center of the bot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // FL
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // FR
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // BL
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2) // BR
        );

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());

    /*

        poseEstimator = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), kinematics,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, Units.degreesToRadians(5)), 
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Units.degreesToRadians(0.01)),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, Units.degreesToRadians(30))
        );
    */

        anglePid.enableContinuousInput(0, 360);
        anglePid.setTolerance(3);

        setDefaultCommand(new DriveCommand(this));

        // Add the encoder readings to shuffleboard
        tab.addNumber("FL angle", () -> frontLeftModule.getAngle().getDegrees());
        tab.addNumber("FR angle", () -> frontRightModule.getAngle().getDegrees());
        tab.addNumber("BL angle", () -> backLeftModule.getAngle().getDegrees());
        tab.addNumber("BR angle", () -> backRightModule.getAngle().getDegrees());
        tab.add(field);

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

    @Override
    public void periodic() {
        // Run the drive command periodically
    }

    /** 
     * Drive the robot with percent output given a ChassisSpeeds object
     * <p>
     * This should be called periodically even if the input is not changing.
     * 
     * @param speeds Chassis speeds with vx and vy <= max linear speed and omega < max rotation speed
    */
    public void drive(ChassisSpeeds speeds, boolean fieldOriented) {
        Gyro gyro = SubsystemFactory.getInstance().getTelemetry().getGyro();
        if(fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond, 
                Rotation2d.fromDegrees(-gyro.getAngle())
            );
        }
            
        if(!Double.isNaN(targetAngle)) {
            speeds.omegaRadiansPerSecond = -anglePid.calculate(gyro.getAngle());
        }
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_LINEAR_SPEED); // Normalize wheel speeds so we don't go faster than 100%
        
        odometry.update(gyro.getRotation2d(), getModuleStates());

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

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

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
        anglePid.setSetpoint(this.targetAngle);

    }

    /**
     * Remove the target angle so that the bot can freely rotate.
     */
    public void removeTargetAngle() {
        targetAngle = Double.NaN;
    }

    public boolean atTargetAngle() {
        return anglePid.atSetpoint();
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

    public void resetLocation(Pose2d botLocation) {
        Pigeon pigeon = (Pigeon) SubsystemFactory.getInstance().getTelemetry().getGyro();
        odometry.resetPosition(botLocation, botLocation.getRotation());
        pigeon.reset(botLocation.getRotation());
    }

    /**
     * 
     * @return Returns PoseEstimator
     */
    public SwerveDriveOdometry getSwerveDriveOdometry(){
        return odometry;
    }
}
