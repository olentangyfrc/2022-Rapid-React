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

public class AutonChooser {
    private ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    private SendableChooser<AutonPath> pathChooser = new SendableChooser<AutonPath>();

    private static Map<AutonPath, String> trajectoryFilePaths = Map.of(
        AutonPath.TEST_PATH, "paths/output/test-path.wpilib.json"
    );


    public AutonChooser() {
        for(AutonPath path : AutonPath.values()) {
            pathChooser.addOption(path.toString(), path);
        }
        tab.add(pathChooser);
    }

    /**
     * Get the selected trajectory from shuffleboard.
     * 
     * @return the selected trajectory
     */
    public Trajectory getTrajectory() throws Exception {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFilePaths.get(pathChooser.getSelected()));
        return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }

    public enum AutonPath {
        TEST_PATH
    }
}
