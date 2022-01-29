package frc.robot.subsystems.drivetrain;

import java.util.HashMap;
import java.util.Map;

// WPILib imports
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
import frc.robot.subsystems.SubsystemFactory;

public class DrivetrainSubsystem extends SubsystemBase {

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
    public void init(Map<String, Integer> portAssignments) throws Exception{
        PortManager portManager = SubsystemFactory.getInstance().getPortManager();

        // Initialize swerve modules
        try {
            frontLeftModule = new SwerveModule(
                portManager.aquirePort(PortType.CAN, portAssignments.get("FL.SwerveMotor"), "FL.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, portAssignments.get("FL.DriveMotor"), "FL.DriveMotor"),
                portManager.aquirePort(PortType.PWM, portAssignments.get("FL.Encoder"), "FL.Encoder"),
                FRONT_LEFT_ANGLE_OFFESET
            );

            frontRightModule = new SwerveModule(
                portManager.aquirePort(PortType.CAN, portAssignments.get("FR.SwerveMotor"), "FR.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, portAssignments.get("FR.DriveMotor"), "FR.DriveMotor"),
                portManager.aquirePort(PortType.PWM, portAssignments.get("FR.Encoder"), "FR.Encoder"),
                FRONT_RIGHT_ANGLE_OFFSET
            );

            backLeftModule = new SwerveModule(
                portManager.aquirePort(PortType.CAN, portAssignments.get("BL.SwerveMotor"), "BL.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, portAssignments.get("BL.DriveMotor"), "BL.DriveMotor"),
                portManager.aquirePort(PortType.PWM, portAssignments.get("BL.Encoder"), "BL.Encoder"),
                BACK_LEFT_ANGLE_OFFSET
            );

            backRightModule = new SwerveModule(
                portManager.aquirePort(PortType.CAN, portAssignments.get("BR.SwerveMotor"), "BR.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, portAssignments.get("BR.DriveMotor"), "BR.DriveMotor"),
                portManager.aquirePort(PortType.PWM, portAssignments.get("BR.Encoder"), "BR.Encoder"),
                BACK_RIGHT_ANGLE_OFFSET
            );

        } catch (Exception exception) {
            //exception.printStackTrace();
            throw exception;
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
