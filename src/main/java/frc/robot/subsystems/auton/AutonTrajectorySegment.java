// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auton;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class AutonTrajectorySegment {
    private Trajectory wpiTrajectory;
    private Rotation2d endAngle; // The desired angle at the end of this segment.
    private Pose2d start;
    private Pose2d end;

    private Rotation2d referenceAngle;

    /**
     * Generate a trajectory between the given start and end point.
     * 
     * @param start The starting position
     * @param end The ending position
     * @param config The trajectory configuration
     * @return The generated trajectory
     */
    public AutonTrajectorySegment(Pose2d start, Pose2d end, TrajectoryConfig config) {
        // Relative coordinates of the end position in reference to the starting position.
        Pose2d relativePosition = end.relativeTo(start);

        referenceAngle = new Rotation2d(Math.atan2(relativePosition.getY(), relativePosition.getX()));
        System.out.println("REFERENCE ANGLE: " + referenceAngle.getDegrees());

        Pose2d angleCorrectedStart = new Pose2d(start.getTranslation(), referenceAngle);
        Pose2d angleCorrectedEnd = new Pose2d(end.getTranslation(), referenceAngle);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(angleCorrectedStart, new ArrayList<Translation2d>(), angleCorrectedEnd, config);
        wpiTrajectory = trajectory;
        endAngle = end.getRotation();
        this.start = start;
        this.end = end;
    }

    /**
     * Get the desired angle at the end of this segment.
     * 
     * @return the desired angle
     */
    public Rotation2d getAngle() {
        return endAngle;
    }

    public Rotation2d getReferenceAngle() {
        return referenceAngle;
    }

    public Trajectory getWpiTrajectory() {
        return wpiTrajectory;
    }

    /**
     * Get the total time in seconds that this trajectory lasts.
     * 
     * @return trajectory's time in seconds
     */
    public double getTotalTimeSeconds() {
        return wpiTrajectory.getTotalTimeSeconds();
    }

    /**
     * Get the start of this segment
     * 
     * @return The start position of this segment in meters
     */
    public Pose2d getStart() {
        return start;
    }

    /**
     * Get the end of this segment
     * 
     * @return The end position in meters
     */
    public Pose2d getEnd() {
        return end;
    }

}
