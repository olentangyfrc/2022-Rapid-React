// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

// Rev Robotics imports:
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

// WPI imports:
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * A swerve module with an angle motor and a drive motor.
 * <p>
 * These are used to form a swerve drivetrain.
 */
public class SwerveModule {
    // Wheel radius in meters
    private static final double WHEEL_RADIUS = 0.09525;

    // Ticks per revolution of the angle encoder.
    public static final double ANGLE_ENCODER_TICKS = 4096;

    // The angle to offset this modules angle by in order to have the bot's forward and this module's forward match
    private double angleOffset;

    // Motors (Make sure these are set to percent output mode)
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    // Encoders
    private AnalogInput angleEncoder;
    private RelativeEncoder driveEncoder;

    // PID controller for wheel angle
    private PIDController anglePid = new PIDController(0.5, 0, 0.0001);

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

        // Make the angle motor turn clockwise with a positive input
        angleMotor.setInverted(true);

        angleEncoder = new AnalogInput(angleEncoderChannel);
        driveEncoder = driveMotor.getEncoder();

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
     * @param angle angle as a rotation2d
     */
    public void setAngle(Rotation2d angle) {
        double output = anglePid.calculate(getAngle().getRadians(), angle.getRadians());
        angleMotor.set(output);
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
     * @return the angle as a rotation2D
     */
    public Rotation2d getAngle() {
        // Raw angle
        double angle = (angleEncoder.getValue() / ANGLE_ENCODER_TICKS * 2 * Math.PI); // Convert rotations to an angle in radians
        // Convert the offset into radians and subtract it from the angle
        angle -= angleOffset * Math.PI / 180;
        // Add a full rotation in radians to make sure the angle is always positive
        angle += 2 * Math.PI;
        // modulo the angle by a full rotation in radians to restrict it to the range [0,2Pi)
        angle %= 2 * Math.PI;

        return new Rotation2d(angle);
    }

    /**
     * Get the speed of the drive motor in meters per second
     * 
     * @return the speed of the drive motor in meters per second
     */
    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    /**
     * Get the current percent output of the drive motor
     * 
     * @return the percent output -1 to 1
     */
    public double getPercentOutput() {
        return driveMotor.get();
    }
}
