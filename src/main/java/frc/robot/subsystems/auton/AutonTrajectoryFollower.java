// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auton;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.drive.Vector2d;

/** Add your docs here. */
public class AutonTrajectoryFollower {
    
    private Trajectory trajectory;
    private HolonomicDriveController driveController;
    private ProfiledPIDController thetaController;
    private Supplier<Pose2d> positionSupplier;
    private Rotation2d targetAngle;

    /**
     * Construct a new AutonTrajectoryFollower
     * 
     * @param xController PID controller to respond to error in x direction
     * @param yController PID controller to respond to error in y direction
     * @param thetaController PID controller to respond to error in angle
     * @param trajectory The trajectory to follow
     * @param positionSupplier A supplier to get the current position of the robot as a Pose2d
     * @param targetAngle The desired angle at the end of the trajectory.
     */
    public AutonTrajectoryFollower(PIDController xController, PIDController yController, ProfiledPIDController thetaController, Trajectory trajectory, Supplier<Pose2d> positionSupplier, Rotation2d targetAngle) {
        this.trajectory = trajectory;
        this.targetAngle = targetAngle;
        this.positionSupplier = positionSupplier;
        this.thetaController = thetaController;

        driveController = new HolonomicDriveController(
            xController,
            yController,
            // This is just to satisfy requirements. We calculate angle on our own.
            new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0))
        );
    }

    /**
     * Calculate the chassis speeds required to follow the trajectory at a given time.
     * <p>
     * These chasses speeds are field oriented.
     * 
     * @param time Time since start in seconds
     * @return The speeds to apply to follow the trajectory
     */
    public ChassisSpeeds calculate(double time) {
        State goal = trajectory.sample(time);
        Pose2d currentPosition = positionSupplier.get();

        // Current position, but with the angle as what the trajectory desires because we control angle separately.
        Pose2d correctedAngle = new Pose2d(currentPosition.getTranslation(), goal.poseMeters.getRotation());

        // Calculate bot-oriented speeds to follow trajectory
        ChassisSpeeds speeds = driveController.calculate(correctedAngle, goal.poseMeters, goal.velocityMetersPerSecond, targetAngle);

        
        // Rotate the vector so that it is field oriented and we don't have to worry about rotation.
        Vector2d translation = new Vector2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond); // bot-oriented
        translation.rotate(-currentPosition.getRotation().getDegrees()); // rotate to field-oriented
        speeds.vxMetersPerSecond = translation.x;
        speeds.vyMetersPerSecond = translation.y;
        
        // Independently calculate theta control
        speeds.omegaRadiansPerSecond = thetaController.calculate(currentPosition.getRotation().getDegrees(), targetAngle.getDegrees());

        return speeds;
    }

    /**
     * Determine if the trajectory has been completed
     * 
     * @param time Time since start in seconds.
     * @return true if the trajectory has been completed.
     */
    public boolean hasFinished(double time) {
        return time >= trajectory.getTotalTimeSeconds();
    }
}
