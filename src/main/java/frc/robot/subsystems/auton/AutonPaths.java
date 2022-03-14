package frc.robot.subsystems.auton;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutonPaths {
    private ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    private SendableChooser<AutonTrajectory> pathChooser = new SendableChooser<AutonTrajectory>();

    private static Map<AutonTrajectory, String> trajectoryFilePaths = Map.of(
        AutonTrajectory.CARGO_3_TO_CARGO_4, "paths/output/cargo3-cargo4.wpilib.json",
        AutonTrajectory.CARGO_4_TO_CARGO_3, "paths/output/cargo4-cargo3.wpilib.json",
        AutonTrajectory.START_1_TO_CARGO_2, "paths/output/start1-cargo2.wpilib.json",
        AutonTrajectory.START2_TO_CARGO_3, "paths/output/start2-cargo3.wpilib.json",
        AutonTrajectory.START3_TO_CARGO4, "paths/output/start3-cargo4.wpilib.json"
    );


    public AutonPaths() {
        for(AutonTrajectory path : AutonTrajectory.values()) {
            pathChooser.addOption(path.toString(), path);
        }
        tab.add(pathChooser);
    }

    /**
     * Get the selected trajectory from shuffleboard.
     * 
     * @return the selected trajectory
     */
    public Trajectory loadTrajectory(AutonTrajectory trajectory) throws Exception {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFilePaths.get(pathChooser.getSelected()));
        return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }

    public enum AutonTrajectory {
        CARGO_3_TO_CARGO_4,
        CARGO_4_TO_CARGO_3,
        START_1_TO_CARGO_2,
        START2_TO_CARGO_3,
        START3_TO_CARGO4
    }
}
