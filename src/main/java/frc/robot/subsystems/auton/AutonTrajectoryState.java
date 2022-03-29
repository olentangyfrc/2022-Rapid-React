// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AutonTrajectoryState {

    // Velocity in meters per second
    private double velocity;
    private Pose2d position;
    // The angle of the segment we are currently on. will be used to convert to field oriented.
    private Rotation2d referenceAngle;

    /**
     * Create a new AutonTrajectoryState
     * 
     * @param velocity Desired velocity in meters per second
     * @param position Desired position in meters
     */
    public AutonTrajectoryState(double velocity, Pose2d position, Rotation2d referenceAngle) {
        this.velocity = velocity;
        this.position = position;
        this.referenceAngle = referenceAngle;
    }

    /**
     * Create a blank state, position 0 and velocity 0.
     */
    public AutonTrajectoryState() {
        this(0, new Pose2d(), new Rotation2d());
    }

    /**
     * Get the desired velocity at this state
     * 
     * @return Desired velocity in meters per second
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Get the desired position at this state
     * 
     * @return The desired position in meters
     */
    public Pose2d getPosition() {
        return position;
    }

    /**
     * Get the angle of the segment we are currently on. will be used to convert to field oriented.
     * <p>
     * Do not try to rotate to this angle! This should only be used for rotation the resulting outputs into field-oriented
     * 
     * @return the reference angle
     */
    public Rotation2d getReferenceAngle() {
        return referenceAngle;
    }
    
}
