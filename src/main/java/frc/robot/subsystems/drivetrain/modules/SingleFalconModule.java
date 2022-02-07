// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * A swerve module using a Falcon 500 as drive motor, a NEO as angle motor, and an encoder for angle.
 * 
 * !!! WIP, needs to be tested. !!!
 * 
 */
public class SingleFalconModule extends SwerveModule {
    // Top speed of drive motor in m/s. May need to be adjusted.
    public static final double MAX_DRIVE_SPEED = 8;

    private WPI_TalonFX driveMotor;
    private CANSparkMax angleMotor;

    private AnalogInput angleEncoder;

    public SingleFalconModule(int angleMotorChannel, int driveMotorChannel, int angleEncoderChannel, double angleOffset) {
        angleMotor = new CANSparkMax(angleMotorChannel, MotorType.kBrushless);
        driveMotor = new WPI_TalonFX(driveMotorChannel);

        angleMotor.setInverted(true);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        driveMotor.config_kP(0, 1);
        driveMotor.config_kI(0, 0);
        driveMotor.config_kD(0, 0);
        driveMotor.config_kF(0, 0);

        driveMotor.configNominalOutputForward(0);
		driveMotor.configNominalOutputReverse(0);
		driveMotor.configPeakOutputForward(1);
		driveMotor.configPeakOutputReverse(-1);
    
        anglePid.setTolerance(0.001);

        angleEncoder = new AnalogInput(angleEncoderChannel);

        anglePid = new PIDController(0.5, 0, 0.0001);
        anglePid.enableContinuousInput(0.0, 2 * Math.PI);

        this.angleOffset = angleOffset;
    }

    @Override
    public void setDrivePercentOutput(double output) {
        driveMotor.set(TalonFXControlMode.Velocity, output * MAX_DRIVE_SPEED);
    }

    @Override
    protected void setAnglePercentOutput(double output) {
        angleMotor.set(output);
    }

    @Override
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

    @Override
    public double getSpeed() {
        return driveMotor.getSelectedSensorVelocity(0);
    }

    /**
     * Do not call this method! It is here to satisfy SwerveModule class.
     * 
     * @hidden
     */
    @Override
    public double getPercentOutput() {
        return Double.NaN;
    }

}
