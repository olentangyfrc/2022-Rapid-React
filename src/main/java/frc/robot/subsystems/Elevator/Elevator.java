package frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;

import java.util.logging.Logger;

public class Elevator extends SubsystemBase{
    private static Logger logger = Logger.getLogger(Elevator.class.getName());

    private final TrapezoidProfile.Constraints SPEED_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 7);
    private WPI_TalonFX winchMotor;
    private final int TALON_CAN = 28;
    private double verticalPercentOutput;
    private int winchVelocity;
    private ElevatorFeedforward elevatorFeedForwardOnBar;
    private ElevatorFeedforward elevatorFeedForwardOffBar;

    private DutyCycleEncoder winchEncoder;
    private double maxHeight;
    private double minHeight;
    private double targetHeight;

    private final double kSOffBar = 0.50046;
    private final double kVOffBar = 0.10773;
    private final double kGOffBar = -0.024405;

    private final double kSOnBar = 0.53251;
    private final double kVOnBar = 4.3071;
    private final double kGOnBar = -0.29183;


    public void init() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();

        //maxVel = 0;
        //maxAcc = 0;

        winchMotor = new WPI_TalonFX(pm.aquirePort(PortType.CAN, TALON_CAN, "Winch Motor"));
        winchMotor.setNeutralMode(NeutralMode.Brake);

        winchEncoder = new DutyCycleEncoder(pm.aquirePort(PortType.DIGITAL, 0, "Winch Encoder"));
        winchEncoder.reset();
        winchEncoder.setDistancePerRotation(1);
        maxHeight = -10;
        minHeight = 0;

        verticalPercentOutput = 0.75;
        
        /*actuatorLengthInPercent = 0.5;
        rightActuatorLengthInPosition = 2;
        leftActuatorLengthInPosition = 2;
        */

        winchVelocity = 0;
        elevatorFeedForwardOffBar = new ElevatorFeedforward(kSOffBar, kGOffBar, kVOffBar);
        elevatorFeedForwardOnBar = new ElevatorFeedforward(kSOnBar, kGOnBar, kVOnBar);
        //winchMotor.setSelectedSensorPosition(0);
    }

    public void extendArms() {
        winchMotor.set(verticalPercentOutput);
    }

    public void retractArms() {
        winchMotor.set(-verticalPercentOutput);
    }

    public void stopWinch(){
        winchMotor.stopMotor();
    }

    public void setVerticalPercentOutput(double output) {
        if(output <= 1 && output >= -1) {
            verticalPercentOutput = output;
        }
    }

    public void getVerticalPercentOutput() {
        winchMotor.get();
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

    public void setVelocityOnWinchMotorOffBar(double vel){
        winchMotor.setVoltage(elevatorFeedForwardOffBar.calculate(vel));
    }

    public void setVelocityOnWinchMotorOnBar(double vel){
        winchMotor.setVoltage(elevatorFeedForwardOnBar.calculate(vel));
    }

    public double getMaxSpeedOffBar(double accel){
        return elevatorFeedForwardOffBar.maxAchievableVelocity(12, accel);
    }

    public double getMaxSpeedOnBar(double accel){
        return elevatorFeedForwardOnBar.maxAchievableVelocity(12, accel);
    }

    public TrapezoidProfile.State getState(){
        return new TrapezoidProfile.State(getEncoderRotations(), getVelocity());
    }

    public double getEncoderRotations(){
        return winchMotor.getSelectedSensorPosition() / 81920.0;
    }

    public double getEncoderPosition(){
        return winchMotor.getSelectedSensorPosition();
    }

    public double getVelocity(){
        return winchMotor.getSelectedSensorVelocity() / 81920.0;
    }
}


