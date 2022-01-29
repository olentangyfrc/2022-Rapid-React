// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemtry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class NavX {
    private AHRS ahrs;

    public void init() throws Exception{
        ahrs = new AHRS(SPI.Port.kMXP);
        
    }
}
