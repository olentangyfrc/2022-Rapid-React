package frc.robot.subsystems.shooter;

import java.util.logging.Logger;

// CTRE imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.SubsystemFactory.BotType;

/**
 * This subsystem controls the flywheel and trigger wheel of the robot.
 */
public class ShooterSubsystem extends SubsystemBase {

    private Logger logger = Logger.getLogger("Subsystem Factory");
    
    private WPI_TalonFX flyWheel;
    private WPI_TalonFX triggerWheel;

    private double targetSpeed = 0;

    // units per rotation of the flywheel encoder
    private final int sensorUnitsPerRotation = 2048;

    // Used to help the flywheel reach a target speed
    private SimpleMotorFeedforward feedForward;
    private PIDController pid;
    
    private PortManager portManager = SubsystemFactory.getInstance().getPortManager();

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    private boolean isStopped = true;
    private boolean isFlywheelAtSetpoint = false;

    /**
     *  Initalizes the shooter subsystem. THIS FUNCTION MUST BE CALLED BEFORE THE SUBSYSTEM WILL WORK!
     * 
     * @param botType The current bot type of the bot that the shooter subsystem is being initialized on. ex: prototype bot or competition bot
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
        pid = new PIDController(12, 0, 0.2);
        pid.setTolerance(1.2);
        flyWheel.setSelectedSensorPosition(0);

        tab.addNumber("Current Shooter Speed", this::getFlySpeed);
        tab.addNumber("Target shooter speed", () -> {
            return targetSpeed;
        });
    }

    /**
     * This function must be called periodically in order for the shooter to run properly
     * <p>
     * This function sets the voltage of both motors constantly unless the shooter is stopped
     */
    public void periodic() {
        if(isStopped) {
            flyWheel.setVoltage(0.0);
            isFlywheelAtSetpoint = false;
        } else {
            double targetVolts = -feedForward.calculate(targetSpeed, pid.calculate(getFlySpeed()));
            flyWheel.setVoltage(targetVolts);
            isFlywheelAtSetpoint = pid.atSetpoint();
        }
    }

    /**
     * Stop the shooter flywheel from applying voltage (it will take a while to slow down)
     */
    public void stop() {
        isStopped = true;
    }

    /**
     * Get the current of the trigger wheel motor
     * 
     * @return
     */
    public double getTriggerCurrent() {
        return SubsystemFactory.getInstance().getPdp().getCurrent(8);
    }
    
    /**
     * Sets the target speed of the fly wheel
     * 
     * @param targetSpeed The target speed in rps
     */
    public void setSpeed(double targetSpeed) {
        isStopped = false;
        pid.setSetpoint(targetSpeed);
        this.targetSpeed = targetSpeed;
    }

    /**
     * Get the speed of the flywheel
     * 
     * @return The current speed of the fly wheel in rps
     */
    public double getFlySpeed() {
        return -flyWheel.getSelectedSensorVelocity()/sensorUnitsPerRotation*10;
    }

    /**
     * Get the current speed of the trigger wheel
     * 
     * @return The current speed of the trigger wheel
     */
    public double getTriggerSpeed() {
        return triggerWheel.getSelectedSensorVelocity();
    }

    /**
     * Determine if the flywheel is within tolerance of the target speed
     * 
     * @return True if the flywheel is within tolerance of the target speed
     */
    public boolean isReady() {
        return getFlyWheelState().equals(flyWheelState.ready);
    }
    
    /**
     * Determine the current state of the flywheel
     * 
     * @return The active state of the fly wheel
     */
    public flyWheelState getFlyWheelState() {
        if (isFlywheelAtSetpoint && pid.getSetpoint() != 0) { return flyWheelState.ready; }
        else if (!pid.atSetpoint()) { return flyWheelState.intermediate; }
        else { return flyWheelState.off; }
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
        triggerWheel.setVoltage(10);
    }

    /**
     * Set the voltage to apply to the trigger wheel
     * 
     * @param voltage The voltage to apply to the trigger wheel
     */
    public void setTriggerVoltage(double voltage) {
        triggerWheel.setVoltage(voltage);
    }

    /**
     * Sets the trigger wheel's speed to 0
     */
    public void stopTrigger() {
        triggerWheel.setVoltage(0);
    }

    /**
     * Stop the fly wheel
     */
    public void stopFlywheel() {
        setSpeed(0);
        isFlywheelAtSetpoint = false;
    }
    
    /**
     * Used to represent the state of the flywheel
     */
    private enum flyWheelState {
        /**
         * ready means that the flywheel is within tolerance of the target speed
         */
        ready,
        /**
         * intermediate means that the flywheel is currently moving towards its target speed
         */
        intermediate,
        /**
         * off means that the flywheel is off
         */
        off
    }

}
