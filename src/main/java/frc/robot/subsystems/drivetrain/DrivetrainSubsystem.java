package frc.robot.subsystems.drivetrain;

// WPILib imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// Package imports
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.drivetrain.commands.DriveCommand;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.interfaces.OzoneSubsystem;

public class DrivetrainSubsystem extends OzoneSubsystem {

    // Angle offsets to intialize Swerve Modules in degrees
    private static final double FRONT_LEFT_ANGLE_OFFESET = 121.46;
    private static final double FRONT_RIGHT_ANGLE_OFFSET = 36.38;
    private static final double BACK_LEFT_ANGLE_OFFSET = 218.4;
    private static final double BACK_RIGHT_ANGLE_OFFSET = 105.08;

    // Declaring Swerve Modules
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    // Distance from center of wheel to center of wheel across the side of the bot in meters
    public static final double WHEEL_BASE = 0.4445;
    // Distance from center of wheel to center of wheel across the front of the bot in meters
    public static final double TRACK_WIDTH = 0.4445;

    private DriveCommand driveCommand;

    // Used to convert from ChassisSpeeds to SwerveModuleStates
    private SwerveDriveKinematics kinematics;

    ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    /**
     * Initialize the drivetrain subsystem
     */
    @Override
    public void init() {
        PortManager portManager = SubsystemFactory.getInstance().getPortManager();

        // Initialize swerve modules
        try {
            frontLeftModule = new SwerveModule(
                portManager.aquirePort(PortType.CAN, 35, "FL.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, 34, "FL.DriveMotor"),
                portManager.aquirePort(PortType.PWM, 0, "FL.Encoder"),
                FRONT_LEFT_ANGLE_OFFESET
            );

            frontRightModule = new SwerveModule(
                portManager.aquirePort(PortType.CAN, 32, "FR.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, 33, "FR.DriveMotor"),
                portManager.aquirePort(PortType.PWM, 1, "FR.Encoder"),
                FRONT_RIGHT_ANGLE_OFFSET
            );

            backLeftModule = new SwerveModule(
                portManager.aquirePort(PortType.CAN, 36, "BL.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, 37, "BL.DriveMotor"),
                portManager.aquirePort(PortType.PWM, 2, "BL.Encoder"),
                BACK_LEFT_ANGLE_OFFSET
            );

            backRightModule = new SwerveModule(
                portManager.aquirePort(PortType.CAN, 31, "BR.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, 30, "BR.DriveMotor"),
                portManager.aquirePort(PortType.PWM, 3, "BR.Encoder"),
                BACK_RIGHT_ANGLE_OFFSET
            );

        } catch (Exception exception) {
            exception.printStackTrace();
        }

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

    @Override
    public void periodic() {
        // Run the drivecommand periodically
        driveCommand.schedule();
    }

    /** 
     * Drive the robot with percent output given a ChassisSpeeds object
     * <p>
     * This should be called periodically even if the input is not changing.
     * 
     * @param speeds Chasis speeds with vx and vy and omega between -1 and 1
    */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1); // Normalize wheel speeds so we don't go faster than 100%

        // Update SwerveModule states
        frontLeftModule.updateState(states[0]);
        frontRightModule.updateState(states[1]);
        backLeftModule.updateState(states[2]);
        backRightModule.updateState(states[3]);
    }
}
