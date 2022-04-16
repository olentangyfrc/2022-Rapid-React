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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AutonTrajectoryFollower {
    
    private AutonTrajectory trajectory;
    private HolonomicDriveController driveController;
    private PIDController thetaController;
    private PIDController xController;
    private PIDController yController;
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
    public AutonTrajectoryFollower(PIDController xController, PIDController yController, PIDController thetaController, AutonTrajectory trajectory, Supplier<Pose2d> positionSupplier) {
        this.trajectory = trajectory;
        this.positionSupplier = positionSupplier;
        this.thetaController = thetaController;
        this.xController = xController;
        this.yController = yController;
        thetaController.enableContinuousInput(0, 360);

        driveController = new HolonomicDriveController(
            xController,
            yController,
            new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)) // Just to satisfy requirements
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
        AutonTrajectoryState goal = trajectory.sample(time);
        Pose2d currentPosition = positionSupplier.get();

        // Pose2d angleCorrectedPosition = new Pose2d(currentPosition.getTranslation(), goal.getReferenceAngle());

        // // Calculate bot-oriented speeds to follow trajectory
        // ChassisSpeeds speeds = driveController.calculate(angleCorrectedPosition, goal.getPosition(), goal.getVelocity(), goal.getPosition().getRotation());

        
        // // Rotate the vector so that it is field oriented and we don't have to worry about rotation.
        // //Vector2d translation = new Vector2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond); // bot-oriented
        // double desiredVelocity = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
        // // System.out.println("X Translation: " + translation.x);
        // // System.out.println("Y Translation: " + translation.y);
        // // translation.rotate(goal.getReferenceAngle().getDegrees()); // rotate to field-oriented
        // // System.out.println("REFERENCE ANGLE: " + goal.getReferenceAngle().getDegrees());

        // double xCorrection = xController.calculate(currentPosition.getX(), goal.getPosition().getX());
        // double yCorrection = yController.calculate(currentPosition.getY(), goal.getPosition().getY());


        // speeds.vxMetersPerSecond = desiredVelocity * Math.cos(goal.getReferenceAngle().getRadians()) + xCorrection;
        // speeds.vyMetersPerSecond = desiredVelocity * Math.sin(goal.getReferenceAngle().getRadians()) + yCorrection;

        
        SmartDashboard.putNumber("Current X: ", currentPosition.getX());
        SmartDashboard.putNumber("Current Y: ", currentPosition.getY());
        SmartDashboard.putNumber("Current Angle: ", currentPosition.getRotation().getDegrees());
        
        SmartDashboard.putNumber("Target X: ", goal.getPosition().getX());
        SmartDashboard.putNumber("Target Y: ", goal.getPosition().getY());
        SmartDashboard.putNumber("Target Angle: ", goal.getPosition().getRotation().getDegrees());
        // Fake the angle so that robot is facing direction of travel.
        Pose2d angleCorrectedGoal = new Pose2d(goal.getPosition().getTranslation(), goal.getReferenceAngle());
        ChassisSpeeds speeds = driveController.calculate(currentPosition, angleCorrectedGoal, goal.getVelocity(), goal.getPosition().getRotation());
        SmartDashboard.putNumber("X Speed: ", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Y Speed: ", speeds.vyMetersPerSecond);
        speeds.omegaRadiansPerSecond = thetaController.calculate(currentPosition.getRotation().getDegrees(), goal.getPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Theta Speed: ", speeds.omegaRadiansPerSecond);
        
        return speeds;
    }

    /**
     * Determine if the trajectory has been completed
     * 
     * @param time Time since start in seconds.
     * @return true if the trajectory has been completed.
     */
    public boolean hasFinished(double time) {
        return time > trajectory.getTotalTimeSeconds();
    }
}
