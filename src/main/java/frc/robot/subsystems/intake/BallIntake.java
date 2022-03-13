// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallIntake extends SubsystemBase {
  private static final int INTAKE_MOTOR_CAN = 30;
  private static final int PCM_CAN = 2;
  private static final int 

  private TalonSRX intakeMotor;

  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;


  /** Creates a new BallIntake. */
  public BallIntake(int intakeMotorChannel) {
    intakeMotor = new TalonSRX(30)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
