package frc.robot.subsystems.shooter;

// CTRE imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import java.util.logging.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory.BotType;

/**
 * 
 */
public class ShooterSubsystem extends SubsystemBase {

    private Logger logger = Logger.getLogger("Subsystem Factory");
    
    private WPI_TalonFX flyWheel;
    private WPI_TalonFX triggerWheel;

    private double targetSpeed = 0;
    private double previousTriggerSpeed = 0;

    private final int sensorUnitsPerRotation = 2048;

    private SimpleMotorFeedforward feedForward;
    private PIDController pid;
    
    private PortManager portManager = SubsystemFactory.getInstance().getPortManager();

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    private boolean isBallLoaded = false;

    // For tuning purposes only
    // Target speed in rps
    private NetworkTableEntry targetShooterSpeed = tab.add("Target Shooter Speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    /**
     *  Initalizes the shooter subsystem. THIS FUNCTION MUST BE CALLED BEFORE THE SUBSYSTEM WILL WORK!
     * @param botType The current bot type of the bot that the shooter subsytem is being initialized on
     * @throws Exception Throws an exception if the fly wheel and trigger wheel ports are already taken or do not exist
     */

    public void init(BotType botType) throws Exception {
        final double Ks;
        final double Kv;
        final double Ka;
        switch (botType) {
            case COVID:            
                Ks = 0.7329;
                Kv = 0.11151;
                Ka = 0.060377;
                feedForward = new SimpleMotorFeedforward(Ks, Kv, Ka);
                flyWheel = new WPI_TalonFX(portManager.aquirePort(PortType.CAN, 43, "shooterFlyWheel"));
                flyWheel.configFactoryDefault();
                triggerWheel = new WPI_TalonFX(portManager.aquirePort(PortType.CAN, 12, "shooterTriggerWheel"));
                break;
            case RAPID_REACT:
                Ks = 0.51003;
                Kv = 0.10813;
                Ka = 0.028561;
                feedForward = new SimpleMotorFeedforward(Ks, Kv, Ka);
                flyWheel = new WPI_TalonFX(portManager.aquirePort(PortType.CAN, 50, "Fly Wheel"));
                flyWheel.configFactoryDefault();
                triggerWheel = new WPI_TalonFX(portManager.aquirePort(PortType.CAN, 16, "Trigger Wheel"));
                triggerWheel.configFactoryDefault();
                triggerWheel.setInverted(true);
                break;
            default:
                logger.severe("Unknown bot");
                break;
        }
        pid = new PIDController(8, 0, 0.2);
        pid.setTolerance(1);
        flyWheel.setSelectedSensorPosition(0);

        tab.addNumber("Distance from hub: ", () -> {
            // Bot position: 
            Translation2d botPosition = SubsystemFactory.getInstance().getDrivetrain().getSwerveDriveOdometry().getPoseMeters().getTranslation();
            Translation2d hubPosition = new Translation2d(8.23, 4.115);

            return botPosition.getDistance(hubPosition);
        });

        tab.addNumber("Current Shooter Speed", this::getFlySpeed);
    }

    /**
     * This function must be called periodically in order for the shooter to run properly
     * This function sets the voltage of both motors constantly
     */
    public void periodic() {
        setSpeed(targetShooterSpeed.getDouble(0.0));

        double targetVolts = -feedForward.calculate(targetSpeed, pid.calculate(getFlySpeed()));
        flyWheel.setVoltage(targetVolts);
    }

    public void takeInBall() {
        triggerWheel.setVoltage(.75);
    }
    
    /**
     * Sets the speed of the fly wheel
     * @param targetSpeed The target speed in rps
     */
    public void setSpeed(double targetSpeed) {
        pid.setSetpoint(targetSpeed);
        this.targetSpeed = targetSpeed;
    }

    /**
     * @return The current speed of the fly wheel in rps
     */
    public double getFlySpeed() {
        return -flyWheel.getSelectedSensorVelocity()/sensorUnitsPerRotation*10;
    }

    public double getTriggerSpeed() {
        return triggerWheel.getSelectedSensorVelocity();
    }

    public double getPreviousTriggerSpeed() {
        return previousTriggerSpeed;
    }

    public boolean isReady() {
        return getFlyWheelState().equals(flyWheelState.ready);
    }
    
    /**
     * @return The active state of the fly wheel
     */
    public flyWheelState getFlyWheelState() {
        if (pid.atSetpoint() && pid.getSetpoint() != 0) { return flyWheelState.ready; }
        else if (!pid.atSetpoint()) { return flyWheelState.intermediate; }
        else { return flyWheelState.off; }
    }

    /**
     * @return The active state of the trigger wheel
     */
    public triggerWheelState getTriggerWheelState() {
        return getBallLoaded()? triggerWheelState.loaded : triggerWheelState.waiting;
    }

    public boolean getBallLoaded() {
        return isBallLoaded;
    }

    public void setBallLoaded(Boolean isLoaded)  {
        isBallLoaded = isLoaded;
    }

    /**
     * @return The current position of teh fly wheel in it's rotation
     */
    public double getFlyPosition() {
        return flyWheel.getSelectedSensorPosition() % 2048;
    }

    /**
     * Rotates the trigger wheel to move the ball into the fly wheel
     */
    public void shoot() {
        if (getFlyWheelState().equals(flyWheelState.off)) { 
            System.out.println("Returning");
            return; }
        //This will cause problems. You cannot pause the flow of code. Code is impatient. 
        // TODO: Fix this.
        //while (getFlyWheelState().equals(flyWheelState.intermediate)) { System.out.println("Waiting"); }
        triggerWheel.setVoltage(10);
        previousTriggerSpeed = 10;
    }

    /**
     * Sets the trigger wheel's speed to 0
     */
    public void stopTrigger() {
        triggerWheel.setVoltage(0);
        previousTriggerSpeed = 0;
    }

    private enum triggerWheelState {
        loaded,
        waiting
    }
    
    private enum flyWheelState {
        ready,
        intermediate,
        off
    }

}
