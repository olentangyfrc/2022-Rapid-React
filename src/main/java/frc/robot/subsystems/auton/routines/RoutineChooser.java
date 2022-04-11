package frc.robot.subsystems.auton.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.auton.AutonPaths;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RoutineChooser {
    // The name of the shuffleboard tab for auton
    private static final String TAB_NAME = "Auton";

    private SendableChooser<CommandBase> autonChooser;
    
    private NetworkTableEntry xPosEntry = Shuffleboard.getTab(TAB_NAME).add("X Position", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private NetworkTableEntry yPosEntry = Shuffleboard.getTab(TAB_NAME).add("Y Position", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private NetworkTableEntry anglePosEntry = Shuffleboard.getTab(TAB_NAME).add("Bot Angle in Degrees", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    
    public RoutineChooser(SwerveDrivetrain drivetrain, ShooterSubsystem shooter, BallIntake intake, AutonPaths paths) {
        autonChooser = new SendableChooser<CommandBase>();
        autonChooser.setDefaultOption("None", new WaitCommand(0));
        autonChooser.addOption("BlueStartOne_TwoCargo", new BlueStartOne_TwoCargo(drivetrain, intake, shooter, paths));
        autonChooser.addOption("BlueStartTwo_FourCargo", new BlueStartTwo_FourCargo(drivetrain, intake, shooter, paths));
        autonChooser.addOption("BlueStartThree_FourCargo", new BlueStartThree_FourCargo(drivetrain, intake, shooter, paths));
        //autonChooser.addOption("ShootAndMoveBack", new ShootAndMoveBack(drivetrain, shooter, intake, new Pose2d(xPosEntry.getDouble(0.0), yPosEntry.getDouble(0.0), Rotation2d.fromDegrees(anglePosEntry.getDouble(0.0)))));
        autonChooser.addOption("ShootAndMoveBack", new CommandBase() {
            @Override
            public void initialize() {
                new ShootAndMoveBack(drivetrain, shooter, intake, new Pose2d(xPosEntry.getDouble(0.0), yPosEntry.getDouble(0.0), Rotation2d.fromDegrees(anglePosEntry.getDouble(0.0)))).schedule();
            }
        });
        Shuffleboard.getTab(TAB_NAME).add(autonChooser);
    }

    public CommandBase get() {
        return autonChooser.getSelected();
    }
}
