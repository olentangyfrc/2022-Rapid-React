// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auton;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/** Add your docs here. */
public class AutonTrajectory {
    private List<AutonTrajectorySegment> segments;

    /**
     * Generate a swerve trajectory following the given waypoints
     * 
     * @param waypoints The waypoints to follow. x+ is towards the red alliance side of the field
     * @param config The trajectory configuration
     * @return The generated trajectory
     */
    public AutonTrajectory(List<Pose2d> waypoints, TrajectoryConfig config) {
        segments = new ArrayList<AutonTrajectorySegment>();
        for(int i = 0; i < waypoints.size() - 1; i++) {
            segments.add(new AutonTrajectorySegment(waypoints.get(i), waypoints.get(i + 1), config));
        }
    }

    /**
     * Generate a swerve trajectory from a start point to an end point
     * 
     * @param start The position to start in in meters
     * @param end The position to end in in meters.
     * @param config The configuration for the trajectory.
     */
    public AutonTrajectory(Pose2d start, Pose2d end, TrajectoryConfig config) {
        this(List.of(start, end), config);
    }

    /**
     * Get the desired state for this trajectory at a given time.
     * 
     * @param seconds Time in seconds since the bot started following this trajectory.
     * @return The desired state for this trajectory.
     */
    public AutonTrajectoryState sample(double seconds) {
        // Determine which segment of the trajectory we are on:
        AutonTrajectorySegment currentSegment = null;
        // Total seconds in all the previous segments.
        double totalSeconds = 0;
        for(AutonTrajectorySegment segment : segments) {
            if(seconds - totalSeconds <= segment.getTotalTimeSeconds()) {
                currentSegment = segment;
                break;
            } else {
                totalSeconds += segment.getTotalTimeSeconds();
            }
        }
        // If we've already finished the trajectory, return the most recent position with a velocity of 0
        if(currentSegment == null) {
            Pose2d pos = segments.get(segments.size()-1).getEnd();
            return new AutonTrajectoryState(0, pos, new Rotation2d());
        }

        double secondsInTrajectory = seconds - totalSeconds;
        Trajectory.State state = currentSegment.getWpiTrajectory().sample(secondsInTrajectory);
        Pose2d angleCorrectedPose = new Pose2d(state.poseMeters.getTranslation(), currentSegment.getAngle());

        return new AutonTrajectoryState(state.velocityMetersPerSecond, angleCorrectedPose, currentSegment.getAngle());
    }

    /**
     * Get the total time in seconds to follow this trajectory.
     * 
     * @return The total time in seconds to follow this trajectory.
     */
    public double getTotalTimeSeconds() {
        double sum = 0;
        for(AutonTrajectorySegment segment : segments) {
            sum += segment.getTotalTimeSeconds();
        }
        return sum;
    }

    /**
     * Determine if the trajectory has been completed at a given time.
     * 
     * @param timeSeconds The amount of time in seconds since when the bot started following the trajectory.
     * @return Return true if the time is greater than the total time of the trajectory
     */
    public boolean isCompleted(double timeSeconds) {
        return timeSeconds > getTotalTimeSeconds();
    }
    
}
