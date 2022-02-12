package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

public class Climber extends SubsystemBase{
    private static Logger logger = Logger.getLogger(Climber.class.getName());

    private CANSparkMax rightLinearActuator;
    private final int LIN_ACT1_CAN = 6;
    private CANSparkMax leftLinearActuator;
    private final int LIN_ACT2_CAN = 61;
    private final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
    private final CANSparkMax.IdleMode MOTOR_MODE = CANSparkMax.IdleMode.kCoast;
    private final SparkMaxAnalogSensor.Mode POTENTIOMETER_MODE = SparkMaxAnalogSensor.Mode.kAbsolute;

    private SparkMaxAnalogSensor rightPotentiometer, leftPotentiometer;
    private SparkMaxPIDController rightPidController, leftPidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc;

    private int maxForwardPosition;
    private int minBackPosition;

    private WPI_TalonFX talon;
    private final int TALON_CAN = 10;

    private DutyCycleEncoder winchEncoder;
    private int maxHeight;
    private int minHeight;

    public void init() {
        rightLinearActuator = new CANSparkMax(LIN_ACT1_CAN, MOTOR_TYPE);
        leftLinearActuator = new CANSparkMax(LIN_ACT2_CAN, MOTOR_TYPE);
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

        maxForwardPosition = 2;
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
        maxHeight = 15;
        minHeight = 5;
    }

    public void pushArmsForward() {
        rightPidController.setReference(maxForwardPosition, CANSparkMax.ControlType.kPosition);
    }

    public void pullArmsBack() {
        rightPidController.setReference(minBackPosition, CANSparkMax.ControlType.kPosition);
    }

    public void extendArms() {
        while(getWinchPosition() < maxHeight) {
            talon.set(0.1);
        }
    }

    public void retractArms() {
        while(getWinchPosition() > minHeight) {
            talon.set(-0.1);
        }
    }

    public void stopArms(){
        talon.stopMotor();
    }

    public void setVerticalPercentOutput() {
        talon.set(TalonFXControlMode.PercentOutput, 0.1);
    }

    public void getVerticalPercentOutput() {
        talon.get();
    }

    public double getPotentiometer1Position() {
        return rightPotentiometer.getPosition();
    }

    public double getPotentiometer2Position(){
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
}