package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAnalogSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;

import java.util.logging.Logger;

public class Climber extends SubsystemBase{
    //logger
    private static Logger logger = Logger.getLogger(Climber.class.getName());

    //Declaration of right linear actuator.
    private CANSparkMax rightLinearActuator;
    //Right Linear Actuator CAN ID
    private final int RIGHT_LIN_ACT_CAN = 7; //proto: 20 comp: 47
    //Declaration of left linear actuator.
    private CANSparkMax leftLinearActuator;
    //Left Linear Actuator CAN ID
    private final int LEFT_LIN_ACT_CAN = 20; //proto: 7 comp: 44
    //Sets Motor Type to Brushless according to Neo motors.
    private final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
    //Sets right and left linear actuators into break mode.
    private final CANSparkMax.IdleMode MOTOR_MODE = CANSparkMax.IdleMode.kBrake;
    //Sets potentiometers of the linear actuators to be absolute encoders
    private final SparkMaxAnalogSensor.Mode POTENTIOMETER_MODE = SparkMaxAnalogSensor.Mode.kAbsolute;

    //Declaration of Right and Left Potentiometers
    private SparkMaxAnalogSensor rightPotentiometer, leftPotentiometer;
    /*private SparkMaxPIDController rightPidController, leftPidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc;
    */

    //The maximum and minimum positions for the right linear actuator.
    private double maxRightForwardPosition;
    private double minRightBackPosition;

    //The maximum and minimum positions for the left linear actuator.
    private double maxLeftForwardPosition;
    private double minLeftBackPosition;

    private boolean isLatched = false;

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

    private PIDController armsController = new PIDController(130, 0, 0);
    public static final double MAX_ARM_ERROR = 0.05;
    private static final double ARMS_TOLERANCE = 0.01;

    private static final double LEFT_ARM_OFFSET = 0.189; //proto: 0.0763 comp: 
    private static final double RIGHT_ARM_OFFSET = 0.11; //proto: 0.1837 comp:

    public static final double MAX_ARM_POSITION = 0.72; //proto: 0.95 comp: 

    private double targetArmPosition;

    //Initialization of the Climber Subsystems
    public void init() throws Exception {
        logger.info("Setting Up Climber");

        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        rightLinearActuator = new CANSparkMax(pm.aquirePort(PortType.CAN, RIGHT_LIN_ACT_CAN, "Right Linear Actuator"), MOTOR_TYPE);
        leftLinearActuator = new CANSparkMax(pm.aquirePort(PortType.CAN, LEFT_LIN_ACT_CAN, "Left Linear Actuator"), MOTOR_TYPE);
        rightLinearActuator.setIdleMode(MOTOR_MODE);
        leftLinearActuator.setIdleMode(MOTOR_MODE);
        rightPotentiometer = rightLinearActuator.getAnalog(POTENTIOMETER_MODE);
        leftPotentiometer = leftLinearActuator.getAnalog(POTENTIOMETER_MODE);
        rightLinearActuator.restoreFactoryDefaults();
        leftLinearActuator.restoreFactoryDefaults();
        rightPotentiometer.setPositionConversionFactor(1);
        leftPotentiometer.setPositionConversionFactor(1);

        armsController.setTolerance(ARMS_TOLERANCE);
        /*rightPidController = rightLinearActuator.getPIDController();
        leftPidController = leftLinearActuator.getPIDController();

        kP = 0.5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;

        kMaxOutput = 1;
        kMinOutput = -1;
        */

        maxRightForwardPosition = 1.05; //proto: 1.12 comp: 0.838
        minRightBackPosition = 0.0939; //proto: 0.171 comp: 0.111

        maxLeftForwardPosition = 1.12; //proto: 1.05 comp: 0.971
        minLeftBackPosition = 0.171; //proto: 0.0939 comp: 0.24

        /*rightPidController.setP(kP);
        rightPidController.setI(kI);
        rightPidController.setD(kD);
        rightPidController.setIZone(kIz);
        rightPidController.setFF(kFF);
        rightPidController.setOutputRange(kMinOutput, kMaxOutput);

        leftPidController.setP(kP);
        leftPidController.setI(kI);
        leftPidController.setD(kD);
        leftPidController.setIZone(kIz);
        leftPidController.setFF(kFF);
        leftPidController.setOutputRange(kMinOutput, kMaxOutput);
        */

        pinsForward = 1;
        pinsReverse = 0;

        compressor = new Compressor(pm.aquirePort(PortType.CAN, PCMCANID, "Compressor"), PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        // pins = new DoubleSolenoid(PCMCANID, PneumaticsModuleType.CTREPCM, pinsForward, pinsReverse);
        // pins.set(Value.kOff);

        targetArmPosition = 0;
        // letGoOfBar();
        Shuffleboard.getTab("Climber").addNumber("Arm pos", this::getAverageArmPosition);
        Shuffleboard.getTab("Climber").addNumber("Left Arm pos", this::getLeftPotentiometerPosition);
        Shuffleboard.getTab("Climber").addNumber("Right Arm pos", this::getRightPotentiometerPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left arm pos", getLeftPotentiometerPosition());
        SmartDashboard.putNumber("Right arm pos", getRightPotentiometerPosition());
    }

    /*public void pushArmsForward() {
        rightPidController.setReference(maxRightForwardPosition, CANSparkMax.ControlType.kPosition);
        leftPidController.setReference(maxLeftForwardPosition, CANSparkMax.ControlType.kPosition);
    }

    public void pullArmsBack() {
        rightPidController.setReference(minRightBackPosition, CANSparkMax.ControlType.kPosition);
        leftPidController.setReference(minLeftBackPosition, CANSparkMax.ControlType.kPosition);
    }
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

    public boolean armsAtPosition(){
        return armsController.atSetpoint();
    }

    public void setTargetArmPosition(double pos){
        targetArmPosition = pos;
    }

    public void applyArmVoltage(){
        double averagePosition = (getLeftPotentiometerPosition() + getRightPotentiometerPosition()) / 2.0;
        double clampedPosition = MathUtil.clamp(averagePosition, targetArmPosition - MAX_ARM_ERROR, targetArmPosition + MAX_ARM_ERROR);
        double volts = armsController.calculate(clampedPosition, targetArmPosition);
        leftLinearActuator.setVoltage(volts);
        rightLinearActuator.setVoltage(volts);
    }

    public void pullArmsBackWithPercent(){
        rightLinearActuator.set(-0.2);
        leftLinearActuator.set(-0.2);
    }

    public void latchOntoBar(){
        // pins.set(Value.kForward);
        isLatched = true;
    }

    public void letGoOfBar(){
        // pins.set(Value.kReverse);
        isLatched = false;
    }

    public void stopRightLinearActuator(){
        rightLinearActuator.stopMotor();
    }

    public void stopLeftLinearActuator(){
        leftLinearActuator.stopMotor();
    }

    public double getRightPotentiometerPosition() {
        return rightPotentiometer.getPosition() - RIGHT_ARM_OFFSET;
    }

    public double getLeftPotentiometerPosition(){
        return leftPotentiometer.getPosition() - LEFT_ARM_OFFSET;
    }

    public double getRightMaxForwardPosition() {
        return maxRightForwardPosition;
    }

    public double getLeftMaxForwardPosition() {
        return maxLeftForwardPosition;
    }

    public double getRightMinBackPosition() {
        return minRightBackPosition;
    }

    public double getLeftMinBackPosition() {
        return minLeftBackPosition;
    }

    public boolean isLatched() {
        return isLatched;
    }
}