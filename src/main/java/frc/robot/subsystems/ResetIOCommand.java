// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetIOCommand extends InstantCommand {
  private IO io;

  public ResetIOCommand(IO io) {
    this.io = io;
    addRequirements(io);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      io.init();
    } catch(Exception ex) {
      ex.printStackTrace();
    }
  }
}
