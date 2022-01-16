package frc.robot.subsystems.drivetrain;

// Package imports
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
// WPI lib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Other imports
import java.util.logging.Logger;

public class DrivetrainSubsystem extends SubsystemBase {

    // Angle offsets to intialize Swerve Modules in degrees
    private static final double FRONT_LEFT_ANGLE_OFFESET = Math.toDegrees(0.0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = Math.toDegrees(0.0);
    private static final double BACK_LEFT_ANGLE_OFFSET = Math.toDegrees(0.0);
    private static final double BACK_RIGHT_ANGLE_OFFSET = Math.toDegrees(0.0);

    static Logger logger = Logger.getLogger(DrivetrainSubsystem.class.getName());

    // Declaring Swerve Modules
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;
    
    public void init() throws Exception {

        PortManager portManager = SubsystemFactory.getInstance().getPortManager();

        frontRightModule = new SwerveModule(portManager.aquirePort(PortType.CAN, 32, "FR.SwerveMotor"),
        portManager.aquirePort(PortType.CAN, 33, "FR.DriveMotor"),
        portManager.aquirePort(PortType.PWM, 1, "FR.Encoder"),
        FRONT_RIGHT_ANGLE_OFFSET);
    }
}
