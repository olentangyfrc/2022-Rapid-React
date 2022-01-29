// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemtry;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Pigeon {
    private WPI_PigeonIMU imu;

    public Pigeon(int deviceId) {
        this.imu = new WPI_PigeonIMU(deviceId);
    }

    public void init() {
        imu.configFactoryDefault();
        imu.setFusedHeading(0.0);
    }

    /**
     * Get the current rotation of the robot
     * 
     * @return the angle of the robot as a rotation2d
     */
    public Rotation2d getAngle() {
        double angle = imu.getFusedHeading();
        return Rotation2d.fromDegrees(angle);
    }

    /**
     * Reset the angle to 0 degrees
     */
    public void resetAngle() {
        resetAngle(Rotation2d.fromDegrees(0.0));
    }

    /**
     * Reset the gyro angle to the given angle
     * 
     * @param angle the angle to reset to as a rotation2d
     */
    public void resetAngle(Rotation2d angle) {
        imu.setFusedHeading(angle.getDegrees());
    }

    /**
     * Enter gyro calibration mode.
     * <p>
     * Wait at least 4 seconds after calling this to move the bot.
     */
    public void calibrateGyro() {
        imu.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }

    /**
     * Enter accelerometer calibration mode.
     * <p>
     * Make sure that the gyro is completely level and still for 10s after this is called.
     */
    public void calibrateAccelerometer() {
        imu.enterCalibrationMode(CalibrationMode.Accelerometer);
    }

}
