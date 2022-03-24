package frc.robot.subsystems.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.auton.routines.BlueStartOne_TwoCargo;
import frc.robot.subsystems.auton.routines.BlueStartThree_FourCargo;
import frc.robot.subsystems.auton.routines.BlueStartTwo_FourCargo;
import frc.robot.subsystems.auton.routines.RedStartOne_TwoCargo;
import frc.robot.subsystems.auton.routines.RedStartThree_FourCargo;
import frc.robot.subsystems.auton.routines.RedStartTwo_FourCargo;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RoutineChooser {
    private SendableChooser<CommandBase> autonChooser;

    public RoutineChooser(SwerveDrivetrain drivetrain, ShooterSubsystem shooter, BallIntake intake, AutonPaths paths) {
        autonChooser.setDefaultOption("None", new WaitCommand(0));
        autonChooser.addOption("BlueStartOne_TwoCargo", new BlueStartOne_TwoCargo(drivetrain, intake, shooter, paths));
        autonChooser.addOption("BlueStartTwo_FourCargo", new BlueStartTwo_FourCargo(drivetrain, intake, shooter, paths));
        autonChooser.addOption("BlueStartThree_FourCargo", new BlueStartThree_FourCargo(drivetrain, intake, shooter, paths));
        autonChooser.addOption("RedStartOne_TwoCargo", new RedStartOne_TwoCargo(drivetrain, intake, shooter, paths));
        autonChooser.addOption("RedStartTwo_FourCargo", new RedStartTwo_FourCargo(drivetrain, intake, shooter, paths));
        autonChooser.addOption("RedStartThree_FourCargo", new RedStartThree_FourCargo(drivetrain, intake, shooter, paths));
    }

    public CommandBase get() {
        return autonChooser.getSelected();
    }
}
