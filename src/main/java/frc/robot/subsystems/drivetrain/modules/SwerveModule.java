// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

// WPI imports:
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A swerve module with an angle motor and a drive motor.
 * <p>
 * These are used to form a swerve drivetrain.
 */
public abstract class SwerveModule {
    // Wheel radius in meters
    protected static final double WHEEL_RADIUS = 0.09525;

    // Ticks per revolution of the angle encoder.
    protected static final double ANGLE_ENCODER_TICKS = 4096;

    // The angle to offset this modules angle by in order to have the bot's forward and this module's forward match
    protected double angleOffset;

    // PID controller for wheel angle
    protected PIDController anglePid;

    /**
     * Set the angle of the module in radians
     * 
     * @param angle angle as a rotation2d
     */
    public void setAngle(Rotation2d angle) {
        double output = anglePid.calculate(getAngle().getRadians(), angle.getRadians());
        setAnglePercentOutput(output);
    }

    /**
     * Set the percent output for the drive motor
     * 
     * @param output Percent output for the drive motor [-1, 1]
     */
    public abstract void setDrivePercentOutput(double output);

    /**
     * Set the percent output for the angle motor
     * 
     * @param output Percent output for the angle motor [-1, 1]
     */
    protected abstract void setAnglePercentOutput(double output);

    /**
     * Update the module with a new angle and speed.
     * <p>
     * This should be called periodically.
     * 
     * @param newState forward, strafe, and rotation should be [-1, 1]
     */
    public void updateState(SwerveModuleState newState) {
        setAngle(newState.angle);
        setDrivePercentOutput(newState.speedMetersPerSecond);
    }

    /**
     * Get the angle of the swerve module
     * 
     * @return the angle as a rotation2D
     */
    public abstract Rotation2d getAngle();

    /**
     * Get the speed of the drive motor in meters per second
     * 
     * @return the speed of the drive motor in meters per second
     */
    public abstract double getSpeed();

    /**
     * Get the current percent output of the drive motor
     * 
     * @return the percent output -1 to 1
     */
    public abstract double getPercentOutput();
}
