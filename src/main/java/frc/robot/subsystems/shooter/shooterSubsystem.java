package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// CTRE imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import java.util.Map;

import java.util.logging.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory.BotType;

/**
 * 
 */
public class shooterSubsystem {

    private Logger logger = Logger.getLogger("Subsystem Factory");
    
    private WPI_TalonFX flyWheel;
    private WPI_TalonFX triggerWheel;

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
        switch (botType) {
            case COVID:            
                final double Ks = 0.7329;
                final double Kv = 0.11151;
                // orignal double Kv = 0.11397;
                // second modififed final double Kv = 1.1397;
                final double Ka = 0.060377;
                feedForward = new SimpleMotorFeedforward(Ks, Kv, Ka);
                flyWheel = new WPI_TalonFX(portManager.aquirePort(PortType.CAN, 43, "shooterFlyWheel"));
                flyWheel.configFactoryDefault();
                triggerWheel = new WPI_TalonFX(portManager.aquirePort(PortType.CAN, 12, "shooterTriggerWheel"));
                pid = new PIDController(8, 0, 0.2);
                Shuffleboard.getTab("Shooter").add("Shooter PID", pid);
                break;
            default:
                logger.severe("Unknown bot");
                break;
        }
        flyWheel.setSelectedSensorPosition(0);
    }
    
    /**
     * Sets the speed of the fly wheel
     * @param targetSpeed The target speed in rps
     */
    public void setSpeed(double targetSpeed) {
        pid.setSetpoint(targetSpeed);
        double targetVolts = feedForward.calculate(targetSpeed, pid.calculate(getFlySpeed()));
        //double targetVolts = pid.calculate(getFlySpeed(), targetSpeed);
        System.out.println(targetVolts);
        flyWheel.setVoltage(targetVolts);
    }

    /**
     * @return The current speed of the fly wheel in rps
     */
    public double getFlySpeed() {
        return flyWheel.getSelectedSensorVelocity()/sensorUnitsPerRotation*10;
    }
    
    /**
     * @return The active state of the fly wheel
     */
    public shooterState getState() {
        if (pid.atSetpoint()) { return shooterState.ready; }
        else { return shooterState.off; }
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
        while (!getState().equals(shooterState.ready)) {}
        triggerWheel.setVoltage(2);
    }
    
    public enum shooterState {
        ready,
        off
    }

}
