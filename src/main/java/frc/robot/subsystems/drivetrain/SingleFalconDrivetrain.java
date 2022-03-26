// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.PortManager;
import frc.robot.subsystems.PortManager.PortType;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.modules.SingleFalconModule;

/** Add your docs here. */
public class SingleFalconDrivetrain extends SwerveDrivetrain {

    @Override
    public void initializeSwerveModules(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception {

        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        thetaController = new PIDController(0.1, 0, 0);

        PortManager portManager = SubsystemFactory.getInstance().getPortManager();
        // Initialize swerve modules
        frontLeftModule = new SingleFalconModule(
            portManager.aquirePort(PortType.CAN, portAssignments.get("FL.SwerveMotor"), "FL.SwerveMotor"),
            portManager.aquirePort(PortType.CAN, portAssignments.get("FL.DriveMotor"), "FL.DriveMotor"),
            portManager.aquirePort(PortType.ANALOG, portAssignments.get("FL.Encoder"), "FL.Encoder"),
            wheelOffsets.get("FL"),
            MAX_LINEAR_SPEED
        );

        frontRightModule = new SingleFalconModule(
            portManager.aquirePort(PortType.CAN, portAssignments.get("FR.SwerveMotor"), "FR.SwerveMotor"),
            portManager.aquirePort(PortType.CAN, portAssignments.get("FR.DriveMotor"), "FR.DriveMotor"),
            portManager.aquirePort(PortType.ANALOG, portAssignments.get("FR.Encoder"), "FR.Encoder"),
            wheelOffsets.get("FR"),
            MAX_LINEAR_SPEED
        );

        backLeftModule = new SingleFalconModule(
            portManager.aquirePort(PortType.CAN, portAssignments.get("BL.SwerveMotor"), "BL.SwerveMotor"),
            portManager.aquirePort(PortType.CAN, portAssignments.get("BL.DriveMotor"), "BL.DriveMotor"),
            portManager.aquirePort(PortType.ANALOG, portAssignments.get("BL.Encoder"), "BL.Encoder"),
            wheelOffsets.get("BL"),
            MAX_LINEAR_SPEED
        );

        backRightModule = new SingleFalconModule(
            portManager.aquirePort(PortType.CAN, portAssignments.get("BR.SwerveMotor"), "BR.SwerveMotor"),
            portManager.aquirePort(PortType.CAN, portAssignments.get("BR.DriveMotor"), "BR.DriveMotor"),
            portManager.aquirePort(PortType.PWM, portAssignments.get("BR.Encoder"), "BR.Encoder"),
            wheelOffsets.get("BR"),
            MAX_LINEAR_SPEED
        );

    }
}
