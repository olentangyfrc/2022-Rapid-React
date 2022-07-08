package frc.robot.subsystems.Climber;

import java.util.logging.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAnalogSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory;

/**
 * This subsystem handles the forwards and backwards component of the climbing arms by controlling the linear actuators.
 */
public class Climber extends SubsystemBase{
    private static Logger logger = Logger.getLogger(Climber.class.getName());

    private CANSparkMax rightLinearActuator;
    private final int RIGHT_LIN_ACT_CAN = 7;
    private CANSparkMax leftLinearActuator;
    private final int LEFT_LIN_ACT_CAN = 20;
    private final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
    private final CANSparkMax.IdleMode MOTOR_MODE = CANSparkMax.IdleMode.kBrake;
    //Sets potentiometers of the linear actuators to be absolute encoders
    private final SparkMaxAnalogSensor.Mode POTENTIOMETER_MODE = SparkMaxAnalogSensor.Mode.kAbsolute;

    private SparkMaxAnalogSensor rightPotentiometer, leftPotentiometer;

    //Declaration of Compressor for Pneumatics
    private Compressor compressor;
    //Pnumatics Control Module (PCM) CAN ID
    private final int PCMCANID = 2;
    //Declaration of DoubleSolenoid (Software Controlled Device to Control Air Flow)
    private DoubleSolenoid pins;
    //The ports on the PCM for extending pneumatics (forward) and retracting pneumatics (reverse).
    private int pinsForward;
    private int pinsReverse;

    //Creating a new Shuffleboard Tab
    private ShuffleboardTab tab = Shuffleboard.getTab("Climber");

    //PID controllers for both linear actuators
    private PIDController leftArmController = new PIDController(130, 0, 0);
    private PIDController rightArmController = new PIDController(130, 0, 0);
    //Max error for linear actuators
    public static final double MAX_ARM_ERROR = 0.05;
    //Tolerance for linear actuators
    private static final double ARMS_TOLERANCE = 0.005;

    //Left linear actuator offset for position 0
    private static final double LEFT_ARM_OFFSET = 0.189;
    //Right linear actuator offset for position
    private static final double RIGHT_ARM_OFFSET = 0.11;

    //Max position for linear actuators
    public static final double MAX_ARM_POSITION = 0.72;

    //Target position for linear actuators
    private double targetArmPosition;

    //Initialization of the Climber Subsystems
    public void init() throws Exception {
        logger.info("Setting Up Climber");

        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        rightLinearActuator = new CANSparkMax(pm.aquirePort(PortType.CAN, RIGHT_LIN_ACT_CAN, "Right Linear Actuator"), MOTOR_TYPE);
        leftLinearActuator = new CANSparkMax(pm.aquirePort(PortType.CAN, LEFT_LIN_ACT_CAN, "Left Linear Actuator"), MOTOR_TYPE);
        //Sets the linear actuators to brake mode
        rightLinearActuator.setIdleMode(MOTOR_MODE);
        leftLinearActuator.setIdleMode(MOTOR_MODE);
        //Initialization of potentiometers
        rightPotentiometer = rightLinearActuator.getAnalog(POTENTIOMETER_MODE);
        leftPotentiometer = leftLinearActuator.getAnalog(POTENTIOMETER_MODE);
        rightLinearActuator.restoreFactoryDefaults();
        leftLinearActuator.restoreFactoryDefaults();
        rightPotentiometer.setPositionConversionFactor(1);
        leftPotentiometer.setPositionConversionFactor(1);

        //Sets the tolerance for the left and right linear actuators
        leftArmController.setTolerance(ARMS_TOLERANCE);
        rightArmController.setTolerance(ARMS_TOLERANCE);

        //Initialization of compressor
        compressor = new Compressor(pm.aquirePort(PortType.CAN, PCMCANID, "Compressor"), PneumaticsModuleType.CTREPCM);
        //Starts the compressor
        compressor.enableDigital();

        //Want arms to start at position 0
        targetArmPosition = 0;
        Shuffleboard.getTab("Climber").addNumber("Arm pos", this::getAverageArmPosition);
        Shuffleboard.getTab("Climber").addNumber("Left Arm pos", this::getLeftPotentiometerPosition);
        Shuffleboard.getTab("Climber").addNumber("Right Arm pos", this::getRightPotentiometerPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left arm pos", getLeftPotentiometerPosition());
        SmartDashboard.putNumber("Right arm pos", getRightPotentiometerPosition());
    }

    /**
     * Move the linear actuator arms forward with percent output
     */
    public void pushArmsForwardWithPercent(){
        rightLinearActuator.set(0.2);
        leftLinearActuator.set(0.2);
    }

    /**
     * Get the average position of the left and right arms.
     * 
     * @return The average position of the right and left arms.
     */
    public double getAverageArmPosition() {
        return (getRightPotentiometerPosition() + getLeftPotentiometerPosition()) / 2;
    }

    /**
     * Sees if the left linear actuator is at the target position
     * 
     * @return True if the arm is at the target position, false otherwise
     */
    public boolean isLeftArmAtPosiiton() {
        return leftArmController.atSetpoint();
    }

    /**
     * Sees if the right linear actuator is at the target position
     * 
     * @return True if the arm is at the target position, false otherwise
     */
    public boolean isRightArmAtPosition() {
        return rightArmController.atSetpoint();
    }

    /**
     * Checks to see if both linear actuators are at the target position
     * 
     * @return True if the arms are at the target position, false otherwise
     */
    public boolean armsAtPosition(){
        return isLeftArmAtPosiiton() && isRightArmAtPosition();
    }

    /**
     * Sets the target position for the linear actuators
     */
    public void setTargetArmPosition(double pos){
        targetArmPosition = pos;
    }

    /**
     * Caclulates voltage to apply to linear actuators and applies it
     */
    public void applyArmVoltage(){
        double leftPosition = getLeftPotentiometerPosition();
        double rightPosition = getRightPotentiometerPosition();

        // Clamp the error in position so that we can move at a constant speed until we get within MAX_ARM_ERROR of our target
        double leftClampedPosition = MathUtil.clamp(leftPosition, targetArmPosition - MAX_ARM_ERROR, targetArmPosition + MAX_ARM_ERROR);
        double rightClampedPosition = MathUtil.clamp(rightPosition, targetArmPosition - MAX_ARM_ERROR, targetArmPosition + MAX_ARM_ERROR);

        double leftVolts = leftArmController.calculate(leftClampedPosition, targetArmPosition);
        double rightVolts = rightArmController.calculate(rightClampedPosition, targetArmPosition);

        leftLinearActuator.setVoltage(leftVolts);
        rightLinearActuator.setVoltage(rightVolts);
    }

    /**
     * Moves the arms backwards with percent ouput
     */
    public void pullArmsBackWithPercent(){
        rightLinearActuator.set(-0.2);
        leftLinearActuator.set(-0.2);
    }

    /**
     * Stops the right linear actuator
     */
    public void stopRightLinearActuator(){
        rightLinearActuator.stopMotor();
    }

    /**
     * Stops the left linear actuator
     */
    public void stopLeftLinearActuator(){
        leftLinearActuator.stopMotor();
    }

    /**
     * Returns the position of the right linear actuator
     * 
     * @return The position from the right potentiometer
     */
    public double getRightPotentiometerPosition() {
        return rightPotentiometer.getPosition() - RIGHT_ARM_OFFSET;
    }

    /**
     * Returns the position of the left linear actuator
     * 
     * @return The position from the left potentiometer
     */
    public double getLeftPotentiometerPosition(){
        return leftPotentiometer.getPosition() - LEFT_ARM_OFFSET;
    }
}