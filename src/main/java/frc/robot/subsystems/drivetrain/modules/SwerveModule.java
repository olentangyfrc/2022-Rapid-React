// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

import edu.wpi.first.math.MathUtil;
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
    public static final double WHEEL_RADIUS = 0.09525;

    // Ticks per revolution of the angle encoder.
    public static final double ANGLE_ENCODER_TICKS = 4096;

    // Any voltage bellow this will be set to this.
    public static final double NOMINAL_VOLTAGE = 0.9;
    public static final double CORRECTION_LIMIT = 0.001;

    // The angle to offset this modules angle by in order to have the bot's forward and this module's forward match
    public double angleOffset;

    // PID controller for wheel angle
    public PIDController anglePid;

    public double velocityConversionFactor;
    public double velocityConversionOffset;
    public PIDController velocityFactorPID;

    public double maxSpeed;

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
     * Set the percent output for the angle motor
     * 
     * @param output Percent output for the angle motor [-1, 1]
     */
    public abstract void setAnglePercentOutput(double output);

    /**
     * Set the velocity of the drive motor
     * 
     * @param velocity
     */
    public void setDriveVelocity(double velocity) {
        // Make sure we don't go faster than we can.
        velocity = MathUtil.clamp(velocity, -maxSpeed, maxSpeed);
        // Calculate voltage to apply in order to achieve the desired velocity
        double voltage = velocity * velocityConversionFactor + Math.copySign(velocityConversionOffset, velocity);

        // Don't apply voltage less than a certain value.
        if(Math.abs(voltage) < NOMINAL_VOLTAGE) {
            voltage = 0;
        } else {
            // Do not affect conversion factor if below nominal voltage.
            velocityConversionFactor += velocityFactorPID.calculate(MathUtil.clamp(Math.abs(getVelocity()) - Math.abs(velocity), -CORRECTION_LIMIT, CORRECTION_LIMIT));
        }

        setDriveVoltage(voltage);
    }

    /**
     * Set the voltage of the drive motor
     * <p>
     * Use setDriveVelocity to set the speed, not this!
     * 
     * @param voltage
     */
    public abstract void setDriveVoltage(double voltage);

    /**
     * Update the module with a new angle and speed.
     * <p>
     * This should be called periodically.
     * 
     * @param newState The new swerve module state
     */
    public void updateState(SwerveModuleState newState) {
        setAngle(newState.angle);
        setDriveVelocity(newState.speedMetersPerSecond);
    }

    /**
     * Get the current state of the module
     * 
     * @return the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(-getAngle().getDegrees()));
    }

    /**
     * Get the angle of the swerve module
     * 
     * @return the angle as a rotation2D
     */
    public abstract Rotation2d getAngle();

    /**
     * Get the velocity of the drive motor in meters per second
     * 
     * @return the speed of the drive motor in meters per second
     */
    public abstract double getVelocity();
}
