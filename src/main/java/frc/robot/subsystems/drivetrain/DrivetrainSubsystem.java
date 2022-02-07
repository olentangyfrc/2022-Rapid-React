package frc.robot.subsystems.drivetrain;

import java.util.Map;
import java.util.logging.Logger;

// WPILib imports
import edu.wpi.first.math.controller.PIDController;
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

public class DrivetrainSubsystem extends SubsystemBase {

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

    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    private Logger logger = Logger.getLogger("DrivetrainSubsystem");

    // PID controller to prevent unintentional rotation when there is no rotation input. Takes in degrees nd outputs percent output.
    private PIDController anglePid = new PIDController(0.004, 0, 0);

    // To be used as the setpoint for the angle PID controller. This is in degrees.
    private double storedRotation;

    /**
     * Initialize the drivetrain subsystem
     */
    public void init(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception{
        PortManager portManager = SubsystemFactory.getInstance().getPortManager();
        // Initialize swerve modules
        try {
            frontLeftModule = new CANSparkMaxModule(
                portManager.aquirePort(PortType.CAN, portAssignments.get("FL.SwerveMotor"), "FL.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, portAssignments.get("FL.DriveMotor"), "FL.DriveMotor"),
                portManager.aquirePort(PortType.PWM, portAssignments.get("FL.Encoder"), "FL.Encoder"),
                wheelOffsets.get("FL")
            );

            frontRightModule = new CANSparkMaxModule(
                portManager.aquirePort(PortType.CAN, portAssignments.get("FR.SwerveMotor"), "FR.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, portAssignments.get("FR.DriveMotor"), "FR.DriveMotor"),
                portManager.aquirePort(PortType.PWM, portAssignments.get("FR.Encoder"), "FR.Encoder"),
                wheelOffsets.get("FR")
            );

            backLeftModule = new CANSparkMaxModule(
                portManager.aquirePort(PortType.CAN, portAssignments.get("BL.SwerveMotor"), "BL.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, portAssignments.get("BL.DriveMotor"), "BL.DriveMotor"),
                portManager.aquirePort(PortType.PWM, portAssignments.get("BL.Encoder"), "BL.Encoder"),
                wheelOffsets.get("BL")
            );

            backRightModule = new CANSparkMaxModule(
                portManager.aquirePort(PortType.CAN, portAssignments.get("BR.SwerveMotor"), "BR.SwerveMotor"),
                portManager.aquirePort(PortType.CAN, portAssignments.get("BR.DriveMotor"), "BR.DriveMotor"),
                portManager.aquirePort(PortType.PWM, portAssignments.get("BR.Encoder"), "BR.Encoder"),
                wheelOffsets.get("BR")
            );

        } catch (Exception exception) {
            //exception.printStackTrace();
            throw exception;
        }

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

        if(speeds.omegaRadiansPerSecond == 0) {
            // Dont move if we have no input.
            if(speeds.vxMetersPerSecond != 0 || speeds.vyMetersPerSecond != 0) {
                speeds.omegaRadiansPerSecond = -anglePid.calculate(gyro.getAngle(), storedRotation);
            }
        } else {
            storedRotation = gyro.getAngle();
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1); // Normalize wheel speeds so we don't go faster than 100%

        // Update SwerveModule states
        frontLeftModule.updateState(states[0]);
        frontRightModule.updateState(states[1]);
        backLeftModule.updateState(states[2]);
        backRightModule.updateState(states[3]);
    }
}
