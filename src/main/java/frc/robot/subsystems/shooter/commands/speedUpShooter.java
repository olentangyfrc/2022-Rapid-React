package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class speedUpShooter extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    double flyWheelRPS;
    //private NetworkTableEntry targetSpeed = Shuffleboard.getTab("Shooter").add("Target speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    public speedUpShooter(ShooterSubsystem shooterSubsystem, double flyWheelRPS) {
        this.shooterSubsystem = shooterSubsystem;
        this.flyWheelRPS = flyWheelRPS;
    
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double distance = SubsystemFactory.getInstance().getVision().getDistanceFromHub();

        shooterSubsystem.setSpeed(distance * 4.8771 + 29.143);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
