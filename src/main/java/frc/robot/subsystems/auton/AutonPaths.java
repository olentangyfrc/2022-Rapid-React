package frc.robot.subsystems.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class AutonPaths {

    // Blue alliance starts
    private static final Pose2d START_B1 = new Pose2d(6.175, 5.21, Rotation2d.fromDegrees(136.232));
    private static final Pose2d START_B2 = new Pose2d(6.711, 2.518, Rotation2d.fromDegrees(226.81));
    private static final Pose2d START_B3 = new Pose2d(8.331, 1.864, Rotation2d.fromDegrees(270.855));

    // Red alliance starts
    private static final Pose2d START_R1 = new Pose2d(10.379, 3.097, Rotation2d.fromDegrees(315.795));
    private static final Pose2d START_R2 = new Pose2d(9.854, 5.757, Rotation2d.fromDegrees(44.719));
    private static final Pose2d START_R3 = new Pose2d(8.159, 6.411, Rotation2d.fromDegrees(91.086));

    private AutonTrajectory startB1ToCargo3_1;

    private AutonTrajectory startB2ToCargo6_2;
    private AutonTrajectory cargo6ToCargo5_2;
    private AutonTrajectory cargo5ToCargo13_2;

    private AutonTrajectory startB3ToCargo6_short_3;

    private AutonTrajectory startB3ToCargo6_3;
    private AutonTrajectory cargo6ToCargo5_3;
    private AutonTrajectory cargo5ToCargo13_3;
    private AutonTrajectory cargo13ToShootPos_3;

    private AutonTrajectory startR1ToCargo9_4;

    private AutonTrajectory startR2ToCargo12_5;
    private AutonTrajectory cargo12ToCargo11_5;
    private AutonTrajectory cargo11ToCargo14_5;

    private AutonTrajectory startR3ToCargo12_6;
    private AutonTrajectory cargo12ToCargo11_6;
    private AutonTrajectory cargo11ToCargo14_6;

    private AutonTrajectory startB2ToCargo5_7;

    public AutonPaths(TrajectoryConfig config) {
        loadTrajectories(config);
    }

    private void loadTrajectories(TrajectoryConfig config) {
        // Routine 1
        startB1ToCargo3_1 = new AutonTrajectory(
            START_B1,
            new Pose2d(5.499, 5.778, Rotation2d.fromDegrees(133.004)),
            config
        );

        // Routine 2
        startB2ToCargo6_2 = new AutonTrajectory(
            START_B2,
            new Pose2d(7.28, 0.856, Rotation2d.fromDegrees(298.549)),
            config
        );

        startB2ToCargo5_7 = new AutonTrajectory(
            START_B2,
            new Pose2d(5.608, 2.163, Rotation2d.fromDegrees(202.835)),
            config
        );

        cargo6ToCargo5_2 = new AutonTrajectory(
            new Pose2d(7.28, 0.856, Rotation2d.fromDegrees(298.549)),
            new Pose2d(5.532, 1.628, Rotation2d.fromDegrees(147.531)),
            config
        );

        cargo5ToCargo13_2 = new AutonTrajectory(
            new Pose2d(5.532, 1.628, Rotation2d.fromDegrees(147.531)),
            new Pose2d(1.628, 1.488, Rotation2d.fromDegrees(218.879)),
            config
        );

        // Routine 3
        startB3ToCargo6_3 = new AutonTrajectory(List.of(
                START_B3,
                new Pose2d(7.819, 0.776, Rotation2d.fromDegrees(250.957)),
                new Pose2d(5.026, 0.778, Rotation2d.fromDegrees(52.504))
            ),
            config
        );

        startB3ToCargo6_short_3 = new AutonTrajectory(List.of(
                START_B3,
                new Pose2d(7.819, 0.776, Rotation2d.fromDegrees(250.957))
            ),
            config
        );

        cargo6ToCargo5_3 = new AutonTrajectory(List.of(
            new Pose2d(5.026, 0.778, Rotation2d.fromDegrees(52.504)),
            new Pose2d(4.936, 1.58, Rotation2d.fromDegrees(86.525)),
            new Pose2d(3.18, 1.753, Rotation2d.fromDegrees(30.959))),
            config
        );

        cargo5ToCargo13_3 = new AutonTrajectory(List.of(
            new Pose2d(3.18, 1.753, Rotation2d.fromDegrees(30.959)),
            new Pose2d(1.157, 1.53, Rotation2d.fromDegrees(221.924))),
            config
        );

        cargo13ToShootPos_3 = new AutonTrajectory(
            new Pose2d(1.157, 1.53, Rotation2d.fromDegrees(221.924)),
            new Pose2d(1.877, 1.723, Rotation2d.fromDegrees(30.781)),
            config
        );

        //Routine 4
        startR1ToCargo9_4 = new AutonTrajectory(
            START_R1,
            new Pose2d(11.141, 2.346, Rotation2d.fromDegrees(315.808)),
            config
        );

        // Routine 5
        startR2ToCargo12_5 = new AutonTrajectory(
            START_R2,
            new Pose2d(9.264, 7.419, Rotation2d.fromDegrees(118.847)),
            config
        );

        cargo12ToCargo11_5 = new AutonTrajectory(
            new Pose2d(9.264, 7.419, Rotation2d.fromDegrees(118.847)),
            new Pose2d(11.012, 6.679, Rotation2d.fromDegrees(324.097)),
            config
        );

        cargo11ToCargo14_5 = new AutonTrajectory(
            new Pose2d(11.012, 6.679, Rotation2d.fromDegrees(324.097)),
            new Pose2d(15.045, 6.776, Rotation2d.fromDegrees(44.706)),
            config
        );

        // Routine 6
        startR3ToCargo12_6 = new AutonTrajectory(
            START_R3,
            new Pose2d(8.792, 7.376, Rotation2d.fromDegrees(78.209)),
            config
        );

        cargo12ToCargo11_6 = new AutonTrajectory(
            new Pose2d(8.792, 7.376, Rotation2d.fromDegrees(78.209)),
            new Pose2d(10.958, 6.626, Rotation2d.fromDegrees(334.005)),
            config
        );

        cargo11ToCargo14_6 = new AutonTrajectory(
            new Pose2d(10.958, 6.626, Rotation2d.fromDegrees(334.005)),
            new Pose2d(15.045, 6.776, Rotation2d.fromDegrees(44.706)),
            config
        );
    }

    public AutonTrajectory getStartB1ToCargo3_1() {
        return startB1ToCargo3_1;
    }

    public AutonTrajectory getStartB2ToCargo6_2() {
        return startB2ToCargo6_2;
    }

    public AutonTrajectory getCargo6ToCargo5_2() {
        return cargo6ToCargo5_2;
    }

    public AutonTrajectory getCargo5ToCargo13_2() {
        return cargo5ToCargo13_2;
    }

    public AutonTrajectory getStartB3ToCargo6_3() {
        return startB3ToCargo6_3;
    }

    public AutonTrajectory getCargo6ToCargo5_3() {
        return cargo6ToCargo5_3;
    }

    public AutonTrajectory getCargo5ToCargo13_3() {
        return cargo5ToCargo13_3;
    }

    public AutonTrajectory getCargo13ToShootPos_3() {
        return cargo13ToShootPos_3;
    }

    public AutonTrajectory getStartR1ToCargo9_4() {
        return startR1ToCargo9_4;
    }

    public AutonTrajectory getStartR2ToCargo12_5() {
        return startR2ToCargo12_5;
    }

    public AutonTrajectory getCargo12ToCargo11_5() {
        return cargo12ToCargo11_5;
    }

    public AutonTrajectory getCargo11ToCargo14_5() {
        return cargo11ToCargo14_5;
    }

    public AutonTrajectory getStartR3ToCargo12_6() {
        return startR3ToCargo12_6;
    }

    public AutonTrajectory getCargo12ToCargo11_6() {
        return cargo12ToCargo11_6;
    }

    public AutonTrajectory getCargo11ToCargo14_6() {
        return cargo11ToCargo14_6;
    }

    public AutonTrajectory getStartB3ToCargo6_short_3() {
        return startB3ToCargo6_short_3;
    }

    public AutonTrajectory getStartB2ToCargo5_7() {
        return startB2ToCargo5_7;
    }
}
