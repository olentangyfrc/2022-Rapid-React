package frc.robot.subsystems.telemetry;

// Package imports
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory.BotType;

// Java imports
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.interfaces.Gyro;
// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class creates and initializes all of the sensors and stores references to them
 */
public class Telemetry extends SubsystemBase {
    
    // Variables for all sensors
    private Gyro gyro;

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
     *  Initializes COVID Bot sensors
     */
    private void initCOVID() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        Pigeon pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();
        pigeon.setInverted(true);

        gyro = pigeon;
    }
    

    /**
     *  Initializes California Bot sensors
     */
    private void initCALIFORNIA() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        Pigeon pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();

        gyro = pigeon;
    }


    /**
     *  Initializes RIO99 sensors
     */
    private void initRIO99() throws Exception {
    }

    /**
     * @return The active pigeon sensor
     */
    public Gyro getGyro() {
        return gyro;
    }
}
