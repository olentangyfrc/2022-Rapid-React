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

import java.util.logging.Logger;

public class Climber extends SubsystemBase{
    private static Logger logger = Logger.getLogger(Climber.class.getName());

    private CANSparkMax rightLinearActuator;
    private final int RIGHT_LIN_ACT_CAN = 61;
    private CANSparkMax leftLinearActuator;
    private final int LEFT_LIN_ACT_CAN = 6;
    private final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
    private final CANSparkMax.IdleMode MOTOR_MODE = CANSparkMax.IdleMode.kBrake;
    private final SparkMaxAnalogSensor.Mode POTENTIOMETER_MODE = SparkMaxAnalogSensor.Mode.kAbsolute;

    private SparkMaxAnalogSensor rightPotentiometer, leftPotentiometer;
    private SparkMaxPIDController rightPidController, leftPidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc;

    private int maxRightForwardPosition;
    private int minRightBackPosition;

    private int maxLeftForwardPosition;
    private int minLeftBackPosition;

    private WPI_TalonFX talon;
    private final int TALON_CAN = 10;
    private double verticalPercentOutput;

    private DutyCycleEncoder winchEncoder;
    private double maxHeight;
    private double minHeight;

    private double actuatorLengthInPercent;
    private double rightActuatorLengthInPosition;
    private double leftActuatorLengthInPosition;

    private Compressor compressor;
    private final int PCMCANID = 0;
    private DoubleSolenoid rightPin;
    private DoubleSolenoid leftPin;
    private int rightPinForward;
    private int rightPinReverse;
    private int leftPinForward;
    private int leftPinReverse;

    private ShuffleboardTab tab = Shuffleboard.getTab("Climber");

    public void init() {
        rightLinearActuator = new CANSparkMax(RIGHT_LIN_ACT_CAN, MOTOR_TYPE);
        leftLinearActuator = new CANSparkMax(LEFT_LIN_ACT_CAN, MOTOR_TYPE);
        rightLinearActuator.setIdleMode(MOTOR_MODE);
        leftLinearActuator.setIdleMode(MOTOR_MODE);
        rightPotentiometer = rightLinearActuator.getAnalog(POTENTIOMETER_MODE);
        leftPotentiometer = leftLinearActuator.getAnalog(POTENTIOMETER_MODE);
        rightLinearActuator.restoreFactoryDefaults();
        leftLinearActuator.restoreFactoryDefaults();
        rightPotentiometer.setPositionConversionFactor(4);
        rightPotentiometer.setPositionConversionFactor(4);
        rightPidController = rightLinearActuator.getPIDController();
        leftPidController = leftLinearActuator.getPIDController();

        kP = 0.2;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;

        kMaxOutput = 1;
        kMinOutput = -1;

        maxRightForwardPosition = 4;
        minRightBackPosition = 1;

        maxLeftForwardPosition = 4;
        minLeftBackPosition = 1;

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

        talon = new WPI_TalonFX(TALON_CAN);

        winchEncoder = new DutyCycleEncoder(0);
        winchEncoder.reset();
        winchEncoder.setDistancePerRotation(1);
        maxHeight = 10;
        minHeight = 0.5;

        verticalPercentOutput = 0.1;
        
        actuatorLengthInPercent = 0.5;
        rightActuatorLengthInPosition = 2;
        leftActuatorLengthInPosition = 2;

        rightPinForward = 0;
        rightPinReverse = 1;
        leftPinForward = 2;
        leftPinReverse = 3;
        compressor = new Compressor(PCMCANID, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        rightPin = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, rightPinForward, rightPinReverse);
        leftPin = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, rightPinForward, rightPinReverse);
        rightPin.set(Value.kOff);
        leftPin.set(Value.kOff);

        tab.addNumber("Right Potentiometer Position", () -> getRightPotentiometerPosition());
        tab.addNumber("Left Potentiometer Position", () -> getLeftPotentiometerPosition());
    }

    public void pushArmsForward() {
        rightPidController.setReference(maxRightForwardPosition, CANSparkMax.ControlType.kPosition);
        rightPidController.setReference(maxLeftForwardPosition, CANSparkMax.ControlType.kPosition);
    }

    public void pullArmsBack() {
        rightPidController.setReference(minRightBackPosition, CANSparkMax.ControlType.kPosition);
        rightPidController.setReference(minLeftBackPosition, CANSparkMax.ControlType.kPosition);
    }

    public void extendArms() {
        talon.set(verticalPercentOutput);
    }

    public void retractArms() {
        talon.set(-verticalPercentOutput);
    }

    public void latchOntoBar(){
        rightPin.set(Value.kForward);
        leftPin.set(Value.kForward);
    }

    public void letGoOfBar(){
        rightPin.set(Value.kReverse);
        leftPin.set(Value.kReverse);
    }

    public void stopWinch(){
        talon.stopMotor();
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
        talon.get();
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
}