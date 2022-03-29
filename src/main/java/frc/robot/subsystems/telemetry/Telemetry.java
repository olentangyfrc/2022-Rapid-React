package frc.robot.subsystems.telemetry;

// Java imports
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.interfaces.Gyro;
// WPILib imports
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
    private ColorSensor colorSensor;

    private Logger logger = Logger.getLogger("Telemetry");

    private BotType  botType;

    public Telemetry(BotType botType) {
        this.botType = botType;
    }

    /**
     *  Initializes sensors for each robot;
     * @throws Exception if sensors could not be initialized
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

    /**
     *  Initializes RIO1 Bot sensors
     * @throws Exception if sensors could not be initialized
     */
    private void initRIO1() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        Pigeon pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();

        gyro = pigeon;
    }

    /**
     *  Initializes COVID Bot sensors
     * @throws Exception if sensors could not be initialized
     */
    private void initCOVID() throws Exception{
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        Pigeon pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        pigeon.init();

        gyro = pigeon;
    }
    

    /**
     *  Initializes Rapid React Bot sensors
     * @throws Exception if sensors could not be initialized
     */
    private void initRAPID_REACT() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        Pigeon pigeon = new Pigeon(pm.aquirePort(PortType.CAN, 21, "Pigeon IMU"));
        ColorSensor colorSensor = new ColorSensor();
        pigeon.init();

        gyro = pigeon;
        this.colorSensor = colorSensor;
    }


    /**
     *  Initializes RIO99 sensors
     * @throws Exception if sensors could not be initialized
     */
    private void initRIO99() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
    }

    /**
     * @return The active pigeon sensor
     */
    public Gyro getGyro() {
        return gyro;
    }

    /**
     * @return The active color sensor
     */
    public ColorSensor getColorSensor() {
        return colorSensor;
    }
}
