package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class shootBallTeleop extends SequentialCommandGroup {

    private ShooterSubsystem shooter;
    private BallIntake intake;
    
    public shootBallTeleop(SwerveDrivetrain driveTrain,ShooterSubsystem shooterSubsystem, BallIntake intake) {

        addCommands(
            new prepareToShoot(driveTrain, shooterSubsystem, intake),
            new feedBall(shooterSubsystem),
            new WaitUntilCommand(()->false) // Never ends by itself
        );

        this.shooter = shooterSubsystem;
        this.intake = intake;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooter.stop();
        shooter.stopTrigger();
        SubsystemFactory.getInstance().getLeds().setIsShooting(false);
    }

}
