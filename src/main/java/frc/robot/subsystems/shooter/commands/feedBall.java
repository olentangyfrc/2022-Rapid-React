package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.leds.Led_Lights;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class feedBall extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public feedBall(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }
    
    @Override
    public void initialize() {
        shooterSubsystem.shoot();
        SubsystemFactory.getInstance().getLeds().setIsShooting(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
