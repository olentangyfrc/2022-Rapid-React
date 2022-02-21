package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;

import java.util.logging.Logger;

public class Climber extends SubsystemBase{
    private static Logger logger = Logger.getLogger(Climber.class.getName());

    private CANSparkMax rightLinearActuator;
    private final int RIGHT_LIN_ACT_CAN = 20;
    private CANSparkMax leftLinearActuator;
    private final int LEFT_LIN_ACT_CAN = 7;
    private final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
    private final CANSparkMax.IdleMode MOTOR_MODE = CANSparkMax.IdleMode.kBrake;
    private final SparkMaxAnalogSensor.Mode POTENTIOMETER_MODE = SparkMaxAnalogSensor.Mode.kAbsolute;

    private SparkMaxAnalogSensor rightPotentiometer, leftPotentiometer;
    private SparkMaxPIDController rightPidController, leftPidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc;

    private double maxRightForwardPosition;
    private double minRightBackPosition;

    private double maxLeftForwardPosition;
    private double minLeftBackPosition;

    private WPI_TalonFX winchMotor;
    private final int TALON_CAN = 28;
    private double verticalPercentOutput;

    private DutyCycleEncoder winchEncoder;
    private double maxHeight;
    private double minHeight;

    private double actuatorLengthInPercent;
    private double rightActuatorLengthInPosition;
    private double leftActuatorLengthInPosition;

    private Compressor compressor;
    private final int PCMCANID = 2;
    private DoubleSolenoid pins;
    private int pinsForward;
    private int pinsReverse;

    private ShuffleboardTab tab = Shuffleboard.getTab("Climber");

    public void init() throws Exception {
        logger.info("Setting Up Climber");
        logger.info("Setting Up Climber");
        logger.info("Setting Up Climber");
        logger.info("Setting Up Climber");
        logger.info("Setting Up Climber");
        logger.info("Setting Up Climber");
        logger.info("Setting Up Climber");
        logger.info("Setting Up Climber");
        PortManager pm = SubsystemFactory.getInstance().getPortManager();
        rightLinearActuator = new CANSparkMax(pm.aquirePort(PortType.CAN, 20, "Right Linear Actuator"), MOTOR_TYPE);
        leftLinearActuator = new CANSparkMax(pm.aquirePort(PortType.CAN, 7, "Left Linear Actuator"), MOTOR_TYPE);
        rightLinearActuator.setIdleMode(MOTOR_MODE);
        leftLinearActuator.setIdleMode(MOTOR_MODE);
        rightPotentiometer = rightLinearActuator.getAnalog(POTENTIOMETER_MODE);
        leftPotentiometer = leftLinearActuator.getAnalog(POTENTIOMETER_MODE);
        rightLinearActuator.restoreFactoryDefaults();
        leftLinearActuator.restoreFactoryDefaults();
        rightPotentiometer.setPositionConversionFactor(4);
        leftPotentiometer.setPositionConversionFactor(4);
        rightPidController = rightLinearActuator.getPIDController();
        leftPidController = leftLinearActuator.getPIDController();

        kP = 0.15;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;

        kMaxOutput = 1;
        kMinOutput = -1;

        maxRightForwardPosition = 3.62;
        minRightBackPosition = 1.68;

        maxLeftForwardPosition = 3.03;
        minLeftBackPosition = 1.23;

        //maxVel = 0;
        //maxAcc = 0;

        rightPidController.setP(kP);
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

        winchMotor = new WPI_TalonFX(pm.aquirePort(PortType.CAN, 28, "Winch Motor"));

        winchEncoder = new DutyCycleEncoder(pm.aquirePort(PortType.DIGITAL, 0, "Winch Encoder"));
        winchEncoder.reset();
        winchEncoder.setDistancePerRotation(1);
        maxHeight = -2;
        minHeight = -0.5;

        verticalPercentOutput = 0.1;
        
        actuatorLengthInPercent = 0.5;
        rightActuatorLengthInPosition = 2;
        leftActuatorLengthInPosition = 2;

        pinsForward = 0;
        pinsReverse = 1;

        compressor = new Compressor(pm.aquirePort(PortType.CAN, 2, "Compressor"), PneumaticsModuleType.CTREPCM);
        //compressor.enableDigital();
        pins = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, 1, 0);
        pins.set(Value.kOff);
    }

    /*@Override
    public void periodic(){
        tab.addNumber("Right Potentiometer Position", () -> getRightPotentiometerPosition());
        tab.addNumber("Left Potentiometer Position", () -> getLeftPotentiometerPosition());
        tab.addNumber("Right Position Conversion Factor", () -> getRightPositionConversionFactor());
        tab.addNumber("Left Position Conversion Fact", () -> getLeftPositionConversionFactor());
    }
    */

    public void pushArmsForward() {
        rightPidController.setReference(3.62, CANSparkMax.ControlType.kPosition);
        rightPidController.setReference(3.03, CANSparkMax.ControlType.kPosition);
    }

    public void pullArmsBack() {
        rightPidController.setReference(1.68, CANSparkMax.ControlType.kPosition);
        rightPidController.setReference(1.23, CANSparkMax.ControlType.kPosition);
    }

    public void extendArms() {
        winchMotor.set(verticalPercentOutput);
    }

    public void retractArms() {
        winchMotor.set(-verticalPercentOutput);
    }

    public void latchOntoBar(){
        pins.set(Value.kForward);
    }

    public void letGoOfBar(){
        pins.set(Value.kReverse);
    }

    public void stopWinch(){
        winchMotor.stopMotor();
    }

    public void stopRightLinearActuator(){
        rightLinearActuator.stopMotor();
    }

    public void stopLeftLinearActuator(){
        leftLinearActuator.stopMotor();
    }

    public void setVerticalPercentOutput(double output) {
        if(output <= 1 && output >= -1) {
            verticalPercentOutput = output;
        }
    }

    public void getVerticalPercentOutput() {
        winchMotor.get();
    }

    public double getRightPotentiometerPosition() {
        return rightPotentiometer.getPosition();
    }

    public double getLeftPotentiometerPosition(){
        return leftPotentiometer.getPosition();
    }

    public double getWinchPosition() {
        return winchEncoder.getDistance();
    }

    public double getMaxHeight() {
        return maxHeight;
    }

    public double getMinHeight() {
        return minHeight;
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

    public void setLinearActuatorLengthInPercent(double percent){
        if(percent >= 0 && percent <= 1){
            actuatorLengthInPercent = percent;
        }
    }

    public double getLinearActuatorLengthInPercent(){
        return actuatorLengthInPercent;
    }

    public double setAndGetRightLinearActuatorPositionFromPercent(){
        rightActuatorLengthInPosition = actuatorLengthInPercent * (maxRightForwardPosition - minRightBackPosition) + minRightBackPosition;
        return rightActuatorLengthInPosition;
    }

    public double setAndGetLeftLinearActuatorPositionFromPercent(){
        leftActuatorLengthInPosition = actuatorLengthInPercent * (maxLeftForwardPosition - minLeftBackPosition) + minLeftBackPosition;
        return leftActuatorLengthInPosition;
    }

    public double getRightLinearActuatorLengthInPosition(){
        return rightActuatorLengthInPosition;
    }

    public double getLeftLinearActuatorLengthInPosition(){
        return leftActuatorLengthInPosition;
    }

    public double getRightPositionConversionFactor(){
        return 4;
    }

    public double getLeftPositionConversionFactor(){
        return 4;
    }
}