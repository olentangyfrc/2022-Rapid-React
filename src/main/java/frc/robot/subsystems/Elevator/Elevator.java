package frc.robot.subsystems.Elevator;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory;

public class Elevator extends SubsystemBase{
    private static Logger logger = Logger.getLogger(Elevator.class.getName());

    // The maximum position error for the elevatorController in rotations.
    public static final double MAX_ERROR = 0.1;
    // The ticks per revolution of the Falcon 500
    public static final double MOTOR_ENCODER_TICKS = 2048;
    // Gear ratio between the Falcon 500 motor and the output shaft for the elevator.
    public static final double WINCH_GEAR_RATIO = 80;
    private static final int WINCH_MOTOR_CAN = 28;

    // The percent output to use for moving the arms forwards and backwards.
    private double verticalPercentOutput;
    
    private WPI_TalonFX winchMotor;
    private DutyCycleEncoder winchEncoder;

    // The minimum and maximum heights in rotations for the elevator.
    private double minHeight;
    private double maxHeight;
    
    // Constraints for the movement of the elevator.
    private final TrapezoidProfile.Constraints SPEED_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 7);
    // Do not change these directly! Use SysID.
    private ProfiledPIDController elevatorController = new ProfiledPIDController(57.839, 0, 5.2621, SPEED_CONSTRAINTS);


    public void init() throws Exception {
        PortManager pm = SubsystemFactory.getInstance().getPortManager();

        //maxVel = 0;
        //maxAcc = 0;

        winchMotor = new WPI_TalonFX(pm.aquirePort(PortType.CAN, WINCH_MOTOR_CAN, "Winch Motor"));
        winchMotor.setNeutralMode(NeutralMode.Brake);

        winchEncoder = new DutyCycleEncoder(pm.aquirePort(PortType.DIGITAL, 0, "Winch Encoder"));
        winchEncoder.reset();

        minHeight = 0;
        maxHeight = 10;

        verticalPercentOutput = 0.75;
        
        // Set the initial goal to the current position.
        setTargetRotations(getPosition());
        
        /*actuatorLengthInPercent = 0.5;
        rightActuatorLengthInPosition = 2;
        leftActuatorLengthInPosition = 2;
        */
    }

    @Override
    public void periodic() {
        // Constantly try to adhere to our target position.

        double targetPosition = elevatorController.getSetpoint().position;
        // Make sure the error is not greater than MAX_ERROR
        double clampedError = MathUtil.clamp(getPosition(), targetPosition - MAX_ERROR, targetPosition + MAX_ERROR);

        winchMotor.setVoltage(elevatorController.calculate(clampedError));
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

    /**
     * Get the maximum height of the elevator
     * 
     * @return the maximum height
     */
    public double getMaxHeight() {
        return maxHeight;
    }

    /**
     * Get the minimum height of the elevator
     * 
     * @return the minimum height
     */
    public double getMinHeight() {
        return minHeight;
    }

    /**
     * Set the target position of the elevator.
     * 
     * @param targetRotations The target position in rotations (up is positive)
     */
    public void setTargetRotations(double targetRotations) {
        elevatorController.setGoal(targetRotations);
    }

    /**
     * Get the position of the elevator in rotations (up is positive)
     * 
     * @return the position of the elevator in rotations
     */
    public double getPosition(){
        // Return the inverted encoder position so that up is positive.
        return -winchEncoder.get();
    }

    /**
     * Get the current velocity of the winch motor in rotations per second.
     * <p>
     * This uses the integrated sensor as it has better support for velocity.
     * 
     * @return
     */
    public double getVelocity(){
        return winchMotor.getSelectedSensorVelocity() / MOTOR_ENCODER_TICKS / WINCH_GEAR_RATIO;
    }
}


