// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * A swerve module using two CANSparkMax motor controllers and an encoder on analog input.
 */
public class CANSparkMaxModule extends SwerveModule {
    // Motors (Make sure these are set to percent output mode)
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    // Encoders
    private AnalogInput angleEncoder;
    private RelativeEncoder driveEncoder;

    /**
     * Initialize a swerve module on the given ports
     * 
     * @param angleMotorChannel CAN port for the angle motor
     * @param driveMotorChannel CAN port for the drive motor
     * @param angleEncoderChannel Analog port for the angle encoder
     * @param angleOffset The angle in degrees by which to offset the angle of the wheel.
     */
    public CANSparkMaxModule(int angleMotorChannel, int driveMotorChannel, int angleEncoderChannel, double angleOffset, double maxSpeed) {
        angleMotor = new CANSparkMax(angleMotorChannel, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);

        anglePid = new PIDController(0.5, 0, 0.0001);
        anglePid.enableContinuousInput(0, 2 * Math.PI);

        // Make the angle motor turn clockwise with a positive input
        angleMotor.setInverted(true);

        angleEncoder = new AnalogInput(angleEncoderChannel);
        driveEncoder = driveMotor.getEncoder();

        // Set the position conversion factor to the circumference of the wheel.
        driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS);

        // Set the velocity conversion factor to the circumference of the wheel
        driveEncoder.setVelocityConversionFactor(2 * Math.PI * WHEEL_RADIUS);

        velocityFactorPID = new PIDController(0.01, 0, 0);
        velocityFactorPID.setSetpoint(0); // We want the error to be 0

        velocityConversionFactor = 1.7799;
        velocityConversionOffset = -0.4126;
        this.maxSpeed = maxSpeed;
        this.angleOffset = angleOffset;
    }

    /**
     * Set the angle of the module in radians
     * 
     * @param angle percent output for angle motor [-1, 1]
     */
    public void setAnglePercentOutput(double output) {
        angleMotor.set(output);
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
    @Override
    public double getVelocity() {
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

    @Override
    public void setDriveVoltage(double voltage) {
        // TODO Auto-generated method stub
        driveMotor.setVoltage(voltage);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        CANSparkMaxModule other = (CANSparkMaxModule) obj;
        if (angleEncoder == null) {
            if (other.angleEncoder != null)
                return false;
        } else if (!angleEncoder.equals(other.angleEncoder))
            return false;
        if (angleMotor == null) {
            if (other.angleMotor != null)
                return false;
        } else if (!angleMotor.equals(other.angleMotor))
            return false;
        if (driveEncoder == null) {
            if (other.driveEncoder != null)
                return false;
        } else if (!driveEncoder.equals(other.driveEncoder))
            return false;
        if (driveMotor == null) {
            if (other.driveMotor != null)
                return false;
        } else if (!driveMotor.equals(other.driveMotor))
            return false;
        return true;
    }
}
