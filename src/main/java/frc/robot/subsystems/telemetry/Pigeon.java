// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * A Pigeon IMU gyroscope
 */
public class Pigeon implements Gyro {
    private WPI_PigeonIMU imu;
    private boolean isInverted = false;

    public Pigeon(int deviceId) {
        this.imu = new WPI_PigeonIMU(deviceId);
    }

    /**
     * Initialize the pigeon
     */
    public void init() {
        imu.setFusedHeading(0.0);
        Shuffleboard.getTab("Drive").addNumber("Gyro angle", this::getAngle);
    }

    /**
     * Get the current rotation of the robot
     * 
     * @return the current heading of the robot in degrees.
     */
    @Override
    public double getAngle() {
        if(isInverted) {
            return -(imu.getFusedHeading() % 360);
        } else {
            return (imu.getFusedHeading() % 360);
        }
    }

    /**
     * Get the heading of the robot as a rotation2d
     * 
     * @return the heading of the robot as a rotation2d
     */
    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * Get the rotational velocity of the bot in degrees per second.
     * 
     * @return
     */
    @Override
    public double getRate() {
        return imu.getRate();
    }

    /**
     * Reset the angle to 0 degrees
     */
    @Override
    public void reset() {
        reset(Rotation2d.fromDegrees(0.0));
    }

    /**
     * Reset the gyro angle to the given angle
     * 
     * @param angle the angle to reset to as a rotation2d
     */
    public void reset(Rotation2d angle) {
        System.out.println("ANGLE: " + angle.getDegrees());
        imu.setFusedHeading(angle.getDegrees() * 64);
    }

    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }

    /**
     * Enter gyro calibration mode.
     * <p>
     * Wait at least 4 seconds after calling this to move the bot.
     */
    @Override
    public void calibrate() {
        while (!imu.getState().equals(PigeonState.Ready));
        imu.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }
    
    /**
     * Enter accelerometer calibration mode.
     * <p>
     * Make sure that the gyro is completely level and still for 10s after this is called.
     */
    public void calibrateAccelerometer() {
        while (!imu.getState().equals(PigeonState.Ready));
        imu.enterCalibrationMode(CalibrationMode.Accelerometer);
    }

    @Override
    public void close() throws Exception {
        imu.close();
    }

}
