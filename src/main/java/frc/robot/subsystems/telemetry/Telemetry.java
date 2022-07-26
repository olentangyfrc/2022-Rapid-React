package frc.robot.subsystems.telemetry;

// Java imports
import java.util.logging.Logger;

// WPILib imports
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Package imports
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.SubsystemFactory.BotType;

/**
 * This class creates and initializes all of the sensors and stores references to them
 */
public class Telemetry extends SubsystemBase {
    
    // Variables for all sensors
    private Gyro gyro;

    private Logger logger = Logger.getLogger("Telemetry");

    private BotType  botType;

    /**
     * Construct a new Telemetry object
     * 
     * @param botType The type of bot that this telemetry object is initialized on
     */
    public Telemetry(BotType botType) {
        this.botType = botType;
    }

    /**
     *  Initializes sensors for each robot;
     * @throws Exception
     */
    public void init() throws Exception {
        switch (botType) {
            case COVID:
                initCOVID();
                break;
            case RAPID_REACT:
                initRAPID_REACT();
                break;
            case RIO99:
                initRIO99();
                break;
            case RIO1:
                initRIO1();
                break;
            default:
                logger.info("Unrecognized Bot");
        }
    }

    private void initRIO1() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        Pigeon pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();

        gyro = pigeon;
    }

    
    /**
     * Initialize COVID bot sensors
     * 
     * @throws Exception If there is an issue assigning a port
     */
    private void initCOVID() throws Exception{
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        Pigeon pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();

        gyro = pigeon;
    }
    

    /**
     *  Initializes Rapid React Bot sensors
     * 
     *  @throws Exception If there is an issue assigning a port
     */
    private void initRAPID_REACT() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        Pigeon pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();

        gyro = pigeon;
    }


    /**
     *  Initializes RIO99 sensors
     * 
     *  @throws Exception If there is an issue assigning a port
     */
    private void initRIO99() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
    }

    /**
     * Get the gyroscope
     * 
     * @return The active pigeon sensor
     */
    public Gyro getGyro() {
        return gyro;
    }
}
