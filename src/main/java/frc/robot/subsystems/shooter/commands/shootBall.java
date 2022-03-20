package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class shootBall extends SequentialCommandGroup {

    private ShooterSubsystem shooter;
    private BallIntake intake;
    
    public shootBall(SwerveDrivetrain driveTrain,ShooterSubsystem shooterSubsystem, BallIntake intake, double flyWheelRPS) {

        addCommands(
            new prepareToShoot(driveTrain, shooterSubsystem, intake, flyWheelRPS),
            new feedBall(shooterSubsystem),
            new WaitCommand(1)
        );

        this.shooter = shooterSubsystem;
        this.intake = intake;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooter.stop();
        shooter.stopTrigger();
        intake.stopNoodleMotor();
        (new takeInBall(shooter)).schedule();
    }

}
