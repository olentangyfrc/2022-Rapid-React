package frc.robot.subsystems.Elevator;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory;

/**
 * The Elevator Subsystem controls the extension and retraction of the climbing arms
 */
public class Elevator extends SubsystemBase{
    private static Logger logger = Logger.getLogger(Elevator.class.getName());

    // The ticks per revolution of the Falcon 500
    public static final double MOTOR_ENCODER_TICKS = 2048;
    // Gear ratio between the Falcon 500 motor and the output shaft for the elevator.
    public static final double WINCH_GEAR_RATIO = 20;

    private static final int WINCH_MOTOR_CAN = 10;

    // The percent output to use for moving the arms forwards and backwards.
    private double verticalPercentOutput;
    
    //TalonFX Motor and Encoder for vertical movement of the arms.
    private WPI_TalonFX winchMotor;
    private DutyCycleEncoder winchEncoder;

    // The minimum and maximum heights in rotations for the elevator.
    private double minHeight;
    private double maxHeight;
    
    // Constraints for the movement of the elevator.
    private final TrapezoidProfile.Constraints SPEED_CONSTRAINTS = new TrapezoidProfile.Constraints(6, 4);
    // Do not change these directly! Use SysID.
    private ProfiledPIDController elevatorController = new ProfiledPIDController(57.839, 0, 0, SPEED_CONSTRAINTS);

    // In manual mode, the elevator does not try to move on its own
    private boolean manualMode = false;


    public void init() throws Exception {
        //PortManager makes sure that a port is not used for two different objects.
        PortManager pm = SubsystemFactory.getInstance().getPortManager();

        //Assigning CAN ID to the Winch Motor.
        winchMotor = new WPI_TalonFX(pm.aquirePort(PortType.CAN, WINCH_MOTOR_CAN, "Winch Motor"));
        //Setting Winch Motor into Brake Mode.
        winchMotor.setNeutralMode(NeutralMode.Brake);

        //Assigning Digital Input Port to the encoder of the Winch Motor.
        winchEncoder = new DutyCycleEncoder(pm.aquirePort(PortType.DIGITAL, 0, "Winch Encoder"));
        //Reseting the position to 0.
        winchEncoder.reset();

        //Minimum height of the arms in rotations (all the way down).
        minHeight = 0;
        //Maximum height of the arms in rotations (all the way up).
        maxHeight = 7.38;

        //The percent output of the winch motor.
        verticalPercentOutput = 0.2; 
        
        // Set the initial goal to the current position so we don't try to move.
        setTargetRotations(getPosition());

        // Display the elevator height to shuffleboard
        Shuffleboard.getTab("Climber").addNumber("Elevator height in rotations", this::getPosition);
    }

    @Override
    public void periodic() {
        // Constantly try to adhere to our target position unless we're in manual mode
        if(!manualMode) {
            setVoltageToWinchMotor();
        }
    }

    /**
     * Sets manual mode
     * 
     * @param manualMode true to enable manual mode, false to disable
     */
    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
        stopWinch();
    }

    /**
     * Causes the arms to start extending
     */
    public void extendArms() {
        winchMotor.set(verticalPercentOutput);
    }

    /**
     * Causes the arms to start retracting
     */
    public void retractArms() {
        winchMotor.set(-verticalPercentOutput);
    }

    /**
     * Stop the winch motor
     */
    public void stopWinch(){
        winchMotor.stopMotor();
    }

    /**
     * Sets the voltage to the elevator using the profiled pid controller
     */
    public void setVoltageToWinchMotor(){
        winchMotor.setVoltage(elevatorController.calculate(getPosition()));
    }

    /**
     * Get the target position of the elevator
     * 
     * @return The target position of the elevator in rotations of the gear
     */
    public double getTargetRotations() {
        return elevatorController.getGoal().position;
    }

    /**
     * Set the percent output of the winch motor.
     * <p>
     * If output is outside the range [-1, 1], no action will be taken.
     * 
     * @param verticalPercentOutput The percent output [-1 to 1]
     */
    public void setVerticalPercentOutput(double output) {
        if(output <= 1 && output >= -1) {
            verticalPercentOutput = output;
        }
    }

    /**
     * Get the percent output of the winch motor.
     * 
     * @return the percent output
     */
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
        elevatorController.reset(getPosition());
        elevatorController.setGoal(targetRotations);
    }

    /**
     * Determine if the elevator has reached its target position
     * 
     * @return true if the elevator has reached its target position
     */
    public boolean isWinchAtGoal() {
       return elevatorController.atGoal();
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
     * This uses the integrated sensor of the winch motor as it has better support for velocity.
     * 
     * @return the velocity of the elevator
     */
    public double getVelocity(){
        return winchMotor.getSelectedSensorVelocity() / MOTOR_ENCODER_TICKS / WINCH_GEAR_RATIO;
    }
}