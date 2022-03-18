package frc.robot.subsystems.shooter;

// CTRE imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import java.util.logging.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    }

    /**
     * This function must be called periodically in order for the shooter to run properly
     * This function sets the voltage of both motors constantly
     */
    public void periodic() {
        double targetVolts = -feedForward.calculate(targetSpeed, pid.calculate(getFlySpeed()));
        flyWheel.setVoltage(targetVolts);
    }

    public void takeInBall() {
        triggerWheel.setVoltage(.75);
    }

    public boolean hasBall() {
        return getTriggerWheelState().equals(triggerWheelState.loaded);        
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
        if (triggerWheel.getSelectedSensorVelocity() == 0 && previousTriggerSpeed != 0) return triggerWheelState.loaded;
        else return triggerWheelState.waiting;
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
        while (getFlyWheelState().equals(flyWheelState.intermediate)) { System.out.println("Waiting"); }
        triggerWheel.setVoltage(5);
        previousTriggerSpeed = 5;
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
