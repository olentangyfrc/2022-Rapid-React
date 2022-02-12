package frc.robot.subsystems.telemetry;

// Package imports
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory.BotType;

// Java imports
import java.util.logging.Logger;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class creates and initializes all of the sensors and stores references to them
 */
public class Telemetry extends SubsystemBase {
    
    // Variables for all sensors
    private Pigeon pigeon;

    private Logger logger = Logger.getLogger("Telemetry");

    private BotType  botType;

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
            case CALIFORNIA:
                initCALIFORNIA();
                break;
            case RIO99:
                initRIO99();
                break;
            default:
                logger.info("Unrecognized Bot");
        }
    }

    /**
     *  Initializes COVID Bot sensors
     */
    private void initCOVID() {}

    /**
     *  Initializes California Bot sensors
     */
    private void initCALIFORNIA() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();
    }


    /**
     *  Initializes RIO99 sensors
     */
    private void initRIO99() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();
    }

    /**
     * @return The active pigeon sensor
     */
    public Pigeon getPigeon() {
        return pigeon;
    }
}
