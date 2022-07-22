package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Prepare to shoot by speeding up our flywheel and turning to the hub, then begin feeding balls to our flywheel
 * <p>
 * This is different from ShootBallAuton because it does not end on its own and must be interrupted.
 */
public class shootBallTeleop extends SequentialCommandGroup {

    private ShooterSubsystem shooter;
    
    public shootBallTeleop(SwerveDrivetrain driveTrain,ShooterSubsystem shooterSubsystem, BallIntake intake) {

        addCommands(
            new prepareToShoot(driveTrain, shooterSubsystem, intake),
            new feedBall(shooterSubsystem),
            new WaitUntilCommand(()->false) // Never ends by itself
        );

        this.shooter = shooterSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooter.stop();
        shooter.stopTrigger();
        SubsystemFactory.getInstance().getLeds().setIsShooting(false);
    }

}
