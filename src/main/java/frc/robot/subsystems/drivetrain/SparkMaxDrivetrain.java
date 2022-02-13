// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Map;

import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.drivetrain.modules.CANSparkMaxModule;

/** Add your docs here. */
public class SparkMaxDrivetrain extends DrivetrainSubsystem {
    @Override
    public void initializeSwerveModules(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception {
        PortManager portManager = SubsystemFactory.getInstance().getPortManager();
        // Initialize swerve modules
        frontLeftModule = new CANSparkMaxModule(
            portManager.aquirePort(PortType.CAN, portAssignments.get("FL.SwerveMotor"), "FL.SwerveMotor"),
            portManager.aquirePort(PortType.CAN, portAssignments.get("FL.DriveMotor"), "FL.DriveMotor"),
            portManager.aquirePort(PortType.PWM, portAssignments.get("FL.Encoder"), "FL.Encoder"),
            wheelOffsets.get("FL")
        );

        frontRightModule = new CANSparkMaxModule(
            portManager.aquirePort(PortType.CAN, portAssignments.get("FR.SwerveMotor"), "FR.SwerveMotor"),
            portManager.aquirePort(PortType.CAN, portAssignments.get("FR.DriveMotor"), "FR.DriveMotor"),
            portManager.aquirePort(PortType.PWM, portAssignments.get("FR.Encoder"), "FR.Encoder"),
            wheelOffsets.get("FR")
        );

        backLeftModule = new CANSparkMaxModule(
            portManager.aquirePort(PortType.CAN, portAssignments.get("BL.SwerveMotor"), "BL.SwerveMotor"),
            portManager.aquirePort(PortType.CAN, portAssignments.get("BL.DriveMotor"), "BL.DriveMotor"),
            portManager.aquirePort(PortType.PWM, portAssignments.get("BL.Encoder"), "BL.Encoder"),
            wheelOffsets.get("BL")
        );

        backRightModule = new CANSparkMaxModule(
            portManager.aquirePort(PortType.CAN, portAssignments.get("BR.SwerveMotor"), "BR.SwerveMotor"),
            portManager.aquirePort(PortType.CAN, portAssignments.get("BR.DriveMotor"), "BR.DriveMotor"),
            portManager.aquirePort(PortType.PWM, portAssignments.get("BR.Encoder"), "BR.Encoder"),
            wheelOffsets.get("BR")
        );
    }
}
