package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Feed a ball from the ball storage area to the flywheel using the trigger wheel
 */
public class feedBall extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    /**
     * Construct a new feedBall command
     * 
     * @param shooterSubsystem The shooter subsystem to feed a ball
     */
    public feedBall(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }
    
    @Override
    public void initialize() {
        shooterSubsystem.shoot();
        SubsystemFactory.getInstance().getLeds().setIsShooting(true);
    }

    /**
     * This command ends immediately.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
    
}
