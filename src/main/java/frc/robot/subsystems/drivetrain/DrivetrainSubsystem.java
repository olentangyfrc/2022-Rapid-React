package frc.robot.subsystems.drivetrain;

// WPILib imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Package imports
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.interfaces.OzoneSubsystem;

public class DrivetrainSubsystem extends OzoneSubsystem {

    // Angle offsets to intialize Swerve Modules in degrees
    private static final double FRONT_LEFT_ANGLE_OFFESET = Math.toDegrees(0.0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = Math.toDegrees(0.0);
    private static final double BACK_LEFT_ANGLE_OFFSET = Math.toDegrees(0.0);
    private static final double BACK_RIGHT_ANGLE_OFFSET = Math.toDegrees(0.0);

    // Declaring Swerve Modules
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    // Wheel base in meters
    public static final double WHEEL_BASE = 0.4445;
    // Track width in meters
    public static final double TRACK_WIDTH = 0.4445;
    
    /**
     *  Initializes SwerveModules
     */

    @Override
    public void init() {

        
        PortManager portManager = SubsystemFactory.getInstance().getPortManager();

        try {
            frontLeftModule = new SwerveModule(portManager.aquirePort(PortType.CAN, 35, "FL.SwerveMotor"),
                                            portManager.aquirePort(PortType.CAN, 34, "FL.DriveMotor"),
                                            portManager.aquirePort(PortType.PWM, 0, "FL.Encoder"),
                                            FRONT_LEFT_ANGLE_OFFESET);

            frontRightModule = new SwerveModule(portManager.aquirePort(PortType.CAN, 32, "FR.SwerveMotor"),
                                                portManager.aquirePort(PortType.CAN, 33, "FR.DriveMotor"),
                                                portManager.aquirePort(PortType.PWM, 1, "FR.Encoder"),
                                                FRONT_RIGHT_ANGLE_OFFSET);

            backLeftModule = new SwerveModule(portManager.aquirePort(PortType.CAN, 36, "BL.SwerveMotor"),
                                            portManager.aquirePort(PortType.CAN, 37, "BL.DriveMotor"),
                                            portManager.aquirePort(PortType.PWM, 2, "BL.Encoder"),
                                            BACK_LEFT_ANGLE_OFFSET);

            backRightModule = new SwerveModule(portManager.aquirePort(PortType.CAN, 31, "BR.SwerveMotor"),
                                            portManager.aquirePort(PortType.CAN, 30, "BR.DriveMotor"),
                                            portManager.aquirePort(PortType.PWM, 3, "BR.Encoder"),
                                            BACK_RIGHT_ANGLE_OFFSET);
        } catch (Exception exception) {
            exception.printStackTrace();
        }
    }

    /**
     * Turns ChassisSpeeds into an array of new SwerveModuleStates
     * 
     * @param speeds Chasis speeds with vx and vy and omega between -1 and 1
     * @return Array of SwerveModuleStates to achieve desired ChassisSpeeds in order of fl, fr, bl, br
     */

    private SwerveModuleState[] getSwerveStates(ChassisSpeeds speeds) {
        double r = Math.hypot(WHEEL_BASE, TRACK_WIDTH);

        double a = speeds.vyMetersPerSecond - speeds.omegaRadiansPerSecond * WHEEL_BASE / r;
        double b = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * WHEEL_BASE / r;
        double c = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * TRACK_WIDTH / r;
        double d = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond * TRACK_WIDTH / r;

        // Percent Output
        double frSpeed = Math.hypot(b, c);
        double flSpeed = Math.hypot(b, d);
        double blSpeed = Math.hypot(a, d);
        double brSpeed = Math.hypot(a, c);

        // Wheel angles in radians
        double frAngle = Math.atan2(b, c);
        double flAngle = Math.atan2(b, d);
        double blAngle = Math.atan2(a, d);
        double brAngle = Math.atan2(a, c);

        // Max of all SwerveModule Speeds
        double max = Math.max(Math.max(flSpeed, frSpeed), Math.max(blSpeed, brSpeed));

        // Normalize wheel speeds to avoid speed overflow
        if (max > 1) {
            frSpeed /= max;
            flSpeed /= max;
            blSpeed /= max;
            brSpeed /= max;
        }

        return new SwerveModuleState[] {
            new SwerveModuleState(flSpeed, new Rotation2d(flAngle)),
            new SwerveModuleState(frSpeed, new Rotation2d(frAngle)),
            new SwerveModuleState(blSpeed, new Rotation2d(blAngle)),
            new SwerveModuleState(brSpeed, new Rotation2d(brAngle))
        };

    }

    /** 
     * Updates swerve modules states
     * 
     * @param speeds Chasis speeds with vx and vy and omega between -1 and 1
    */

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = getSwerveStates(speeds);

        // Update SwerveModule states
        frontLeftModule.updateState(states[0]);
        frontRightModule.updateState(states[1]);
        backLeftModule.updateState(states[2]);
        backRightModule.updateState(states[3]);
    }
}
