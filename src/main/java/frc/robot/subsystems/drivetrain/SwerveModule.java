// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

// WPI imports:
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Rev Robotics imports:
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

/**
 * A swerve module with an angle motor and a drive motor.
 * <p>
 * These are used to form a swerve drivetrain.
 */
public class SwerveModule {
    // Wheel radius in meters
    private static final double WHEEL_RADIUS = 0.09525;

    // The angle by which to offset the angle of the wheel
    private double angleOffset;

    // Motors (Make sure these are set to percent output mode)
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    // Encoders
    private AnalogEncoder angleEncoder;
    private RelativeEncoder driveEncoder;

    // PID controller for wheel angle
    private PIDController anglePid = new PIDController(1, 0, 0); // Default values. Need to be tuned!!!

    /**
     * Initialize a swerve module on the given ports
     * 
     * @param angleMotorChannel CAN port for the angle motor
     * @param driveMotorChannel CAN port for the drive motor
     * @param angleEncoderChannel Analog port for the angle encoder
     * @param angleOffset The angle in degrees by which to offset the angle of the wheel.
     */
    public SwerveModule(int angleMotorChannel, int driveMotorChannel, int angleEncoderChannel, double angleOffset) {
        angleMotor = new CANSparkMax(angleMotorChannel, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);

        angleEncoder = new AnalogEncoder(new AnalogInput(angleEncoderChannel));
        driveEncoder = driveMotor.getEncoder();

        // Set the distance per rotation to one full rotation in radians.
        angleEncoder.setDistancePerRotation(2 * Math.PI);

        // Set the position conversion factor to the circumference of the wheel.
        driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS);

        // Set the velocity conversion factor to the circumference of the wheel
        driveEncoder.setVelocityConversionFactor(2 * Math.PI * WHEEL_RADIUS);

        anglePid.enableContinuousInput(0, 2 * Math.PI);

        this.angleOffset = angleOffset;
    }

    /**
     * Set the angle of the module in radians
     * 
     * @param angle angle in radians
     */
    public void setAngle(Rotation2d angle) {
        anglePid.setSetpoint(angle.getRadians());
        angleMotor.set(anglePid.calculate(getAngle().getRadians()));
    }

    /**
     * Set the percent output for the drive motor
     * 
     * @param output Percent output for the drive motor [-1, 1]
     */
    public void setPercentOutput(double output) {
        driveMotor.set(output);
    }

    /**
     * Update the module with a new angle and speed.
     * <p>
     * This should be called periodically.
     * 
     * @param newState
     */
    public void updateState(SwerveModuleState newState) {
        setAngle(newState.angle);
        setPercentOutput(newState.speedMetersPerSecond);
    }

    /**
     * Get the angle of the swerve module
     * 
     * @return the angle
     */
    public Rotation2d getAngle() {
        // Apply the angle offset and modulo the distance by 2Pi to make the output continuous
        return new Rotation2d((angleEncoder.getDistance() + angleOffset * 180 / Math.PI) % (Math.PI * 2));
    }

    /**
     * Get the speed of the bot in meters per second
     * 
     * @return the speed of the bot in meters per second
     */
    public double getSpeed() {
        return driveEncoder.getVelocity();
    }
}
