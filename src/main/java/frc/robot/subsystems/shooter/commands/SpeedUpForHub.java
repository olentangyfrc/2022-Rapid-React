package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SpeedUpForHub extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public SpeedUpForHub(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double distance = SubsystemFactory.getInstance().getVision().getDistanceFromHub();
        Pose2d pose = SubsystemFactory.getInstance().getDrivetrain().getLocation();

        shooterSubsystem.setSpeed(distance * 4.8771 + 29.143);
        // shooterSubsystem.setSpeed(Math.pow(distance, 2) * 0.3689 + distance * 1.6366 + 35.7479);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
