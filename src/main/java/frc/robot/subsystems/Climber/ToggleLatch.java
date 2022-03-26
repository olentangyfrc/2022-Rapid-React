// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleLatch extends InstantCommand {
  private Climber climber;

  public ToggleLatch(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(climber.isLatched()) {
      climber.letGoOfBar();
    } else {
      climber.latchOntoBar();
    }
  }
}
