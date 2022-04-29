package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootBallAuton extends SequentialCommandGroup {

    private ShooterSubsystem shooter;
    private BallIntake intake;
    
    /**
     * Shoot balls during autonomous
     * 
     * @param driveTrain drivetrain that will rotate to the hub
     * @param shooterSubsystem Shooter to shoot with
     * @param intake
     * @param shootTime Time to allocate for shooting balls. Too small will result in balls not being shot. Too large will result in wasted time.
     */
    public ShootBallAuton(SwerveDrivetrain driveTrain,ShooterSubsystem shooterSubsystem, BallIntake intake, double shootTime) {

        addCommands(
            new prepareToShoot(driveTrain, shooterSubsystem, intake),
            new feedBall(shooterSubsystem),
            new WaitCommand(shootTime)
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
