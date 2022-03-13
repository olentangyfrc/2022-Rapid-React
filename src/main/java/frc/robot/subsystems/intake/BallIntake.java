// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;

public class BallIntake extends SubsystemBase {
  private static final int INTAKE_MOTOR_CAN = 30;
  private static final int UP_PCM = 3;
  private static final int DOWN_PCM = 4;

  private static final double INTAKE_PERCENT_OUTPUT = 0.5;

  private TalonSRX intakeMotor;
  private DoubleSolenoid intakeSolenoid;


  /**
   * Create a new BallIntake
   * 
   * @param intakeMotorChannel the CAN channel of the intake motor.
   */
  public BallIntake(int intakeMotorChannel) throws Exception {
    PortManager pm = SubsystemFactory.getInstance().getPortManager();
    intakeMotor = new TalonSRX(pm.aquirePort(PortType.CAN, INTAKE_MOTOR_CAN, "Intake motor"));

    intakeSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      pm.aquirePort(PortType.PCM, DOWN_PCM, "Intake forward"),
      pm.aquirePort(PortType.PCM, UP_PCM, "Intake back")
    );
  }

  /**
   * Bring the intake down using the pneumatics.
   */
  public void putIntakeDown() {
    intakeSolenoid.set(Value.kForward);
  }

  /**
   * Bring the intake up using the pneumatics
   */
  public void bringIntakeUp() {
    intakeSolenoid.set(Value.kReverse);
  }

  /**
   * Start running the intake motor at the percent output specified in INTAKE_PERCENT_OUTPUT
   */
  public void startIntakeMotor() {
    intakeMotor.set(ControlMode.PercentOutput, INTAKE_PERCENT_OUTPUT);
  }

  /**
   * Stop running the intake motor.
   */
  public void stopIntakeMotor() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}
