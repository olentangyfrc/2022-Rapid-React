package frc.robot.subsystems.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutonChooser {
    private ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    private SendableChooser<AutonPath> pathChooser = new SendableChooser<AutonPath>();

    private static Map<AutonPath, String> trajectoryFilePaths = Map.of(
        AutonPath.TEST_PATH, "\\paths\\output\\test_path.wpilib.json"
    );


    public AutonChooser() {
        for(AutonPath path : AutonPath.values()) {
            pathChooser.addOption(path.toString(), path);
        }
    }

    /**
     * Get the selected trajectory from shuffleboard.
     * 
     * @return the selected trajectory
     */
    public Trajectory getTrajectory() {
        return TrajectoryUtil.deserializeTrajectory(trajectoryFilePaths.get(pathChooser.getSelected()));
    }

    public enum AutonPath {
        TEST_PATH
    }
}
