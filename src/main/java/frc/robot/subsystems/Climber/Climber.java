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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

public class Climber extends SubsystemBase{
    private static Logger logger = Logger.getLogger(Climber.class.getName());

    private CANSparkMax rightLinearActuator;
    private final int LIN_ACT1_CAN = 61;
    private CANSparkMax leftLinearActuator;
    private final int LIN_ACT2_CAN = 6;
    private final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
    private final CANSparkMax.IdleMode MOTOR_MODE = CANSparkMax.IdleMode.kBrake;
    private final SparkMaxAnalogSensor.Mode POTENTIOMETER_MODE = SparkMaxAnalogSensor.Mode.kAbsolute;

    private SparkMaxAnalogSensor rightPotentiometer, leftPotentiometer;
    private SparkMaxPIDController rightPidController, leftPidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc;

    private int maxForwardPosition;
    private int minBackPosition;

    private WPI_TalonFX talon;
    private final int TALON_CAN = 10;
    private double verticalPercentOutput;

    private DutyCycleEncoder winchEncoder;
    private double maxHeight;
    private double minHeight;

    private double actuatorLengthInPercent;
    private double actuatorLengthInPosition;

    private Compressor compressor;
    private final int PCMCANID = 0;
    private DoubleSolenoid rightPin;
    private DoubleSolenoid leftPin;
    private int rightPinForward;
    private int rightPinReverse;
    private int leftPinForward;
    private int leftPinReverse;

    public void init() {
        rightLinearActuator = new CANSparkMax(LIN_ACT1_CAN, MOTOR_TYPE);
        leftLinearActuator = new CANSparkMax(LIN_ACT2_CAN, MOTOR_TYPE);
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

        leftLinearActuator.follow(rightLinearActuator);

        kP = 0.2;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;

        kMaxOutput = 1;
        kMinOutput = -1;

        maxForwardPosition = 4;
        minBackPosition = 1;

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

        verticalPercentOutput = 0.05;
        
        actuatorLengthInPercent = 0.5;
        actuatorLengthInPosition = 2;

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
    }

    public void pushArmsForward() {
        rightPidController.setReference(maxForwardPosition, CANSparkMax.ControlType.kPosition);
    }

    public void pullArmsBack() {
        rightPidController.setReference(minBackPosition, CANSparkMax.ControlType.kPosition);
    }

    public void extendArms() {
        while(getWinchPosition() < maxHeight) {
            talon.set(verticalPercentOutput);
        }
    }

    public void retractArms() {
        while(getWinchPosition() > minHeight) {
            talon.set(verticalPercentOutput);
        }
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

    public double getMaxForwardPosition() {
        return maxForwardPosition;
    }

    public double getMinBackPosition() {
        return minBackPosition;
    }

    public void setLinearActuatorLengthInPercent(double percent){
        if(percent >= 0 && percent <= 1){
            actuatorLengthInPercent = percent;
        }
    }

    public double getLinearActuatorLengthInPercent(){
        return actuatorLengthInPercent;
    }

    public double setAndGetLinearActuatorPositionFromPercent(){
        actuatorLengthInPosition = actuatorLengthInPercent * (maxForwardPosition - minBackPosition) + minBackPosition;
        return actuatorLengthInPosition;
    }

    public double getLinearActuatorLengthInPosition(){
        return actuatorLengthInPosition;
    }
}