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

    private PIDController leftArmController = new PIDController(130, 0, 0);
    private PIDController rightArmController = new PIDController(130, 0, 0);
    public static final double MAX_ARM_ERROR = 0.05;
    private static final double ARMS_TOLERANCE = 0.01;

    private static final double LEFT_ARM_OFFSET = 0.19; //proto: 0.0763 comp: 
    private static final double RIGHT_ARM_OFFSET = 0.08; //proto: 0.1837 comp:

    public static final double MAX_ARM_POSITION = 0.7; //proto: 0.95 comp: 

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

        leftArmController.setTolerance(ARMS_TOLERANCE);
        rightArmController.setTolerance(ARMS_TOLERANCE);

        maxRightForwardPosition = 1.05; //proto: 1.12 comp: 0.838
        minRightBackPosition = 0.0939; //proto: 0.171 comp: 0.111

        maxLeftForwardPosition = 1.12; //proto: 1.05 comp: 0.971
        minLeftBackPosition = 0.171; //proto: 0.0939 comp: 0.24

        pinsForward = 1;
        pinsReverse = 0;

        compressor = new Compressor(pm.aquirePort(PortType.CAN, PCMCANID, "Compressor"), PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        pins = new DoubleSolenoid(PCMCANID, PneumaticsModuleType.CTREPCM, pinsForward, pinsReverse);
        pins.set(Value.kOff);

        targetArmPosition = 0;
        letGoOfBar();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm pos", getAverageArmPosition());
        SmartDashboard.putNumber("Left arm pos", getLeftPotentiometerPosition());
        SmartDashboard.putNumber("Right arm pos", getRightPotentiometerPosition());
    }

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

    public boolean isLeftArmAtPosiiton() {
        return leftArmController.atSetpoint();
    }

    public boolean isRightArmAtPosition() {
        return rightArmController.atSetpoint();
    }

    public boolean armsAtPosition(){
        if (isLeftArmAtPosiiton() && isRightArmAtPosition()) return true;
        return false;
    }

    public void setTargetArmPosition(double pos){
        targetArmPosition = pos;
    }

    public void setArmVoltage(){
        double leftPosition = getLeftPotentiometerPosition();
        double rightPosition = getRightPotentiometerPosition();
        double leftClampedPosition = MathUtil.clamp(leftPosition, targetArmPosition - MAX_ARM_ERROR, targetArmPosition + MAX_ARM_ERROR);
        double rightClampedPosition = MathUtil.clamp(rightPosition, targetArmPosition - MAX_ARM_ERROR, targetArmPosition + MAX_ARM_ERROR);
        double leftVolts = leftArmController.calculate(leftClampedPosition, targetArmPosition);
        double rightVolts = rightArmController.calculate(rightClampedPosition, targetArmPosition);
        leftLinearActuator.setVoltage(leftVolts);
        rightLinearActuator.setVoltage(rightVolts);
    }

    public void pullArmsBackWithPercent(){
        rightLinearActuator.set(-0.2);
        leftLinearActuator.set(-0.2);
    }

    public void latchOntoBar(){
        pins.set(Value.kForward);
        isLatched = true;
    }

    public void letGoOfBar(){
        pins.set(Value.kReverse);
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