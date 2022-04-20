package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;

public class Led_Lights extends SubsystemBase {


  private static Spark m_blinkin = null;

  public static final double RED_DEFAULT_CODE = -0.85;
  public static final double BLUE_DEFAULT_CODE = -0.83;
  public static final double WHITE_DEFAULT_CODE = -0.81;

  public static final double RED_SHOT_CODE = 0.61;
  public static final double BLUE_SHOT_CODE = 0.87;
  public static final double WHITE_SHOT_CODE = 0.93;

  private boolean isShooting = false;
  
  public Led_Lights(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    SmartDashboard.putBoolean("Initalized lights", true);

  }

  @Override
  public void periodic() {
    switch(DriverStation.getAlliance()) {
      case Red:
        if(isShooting) {
          set(RED_SHOT_CODE);
        } else {
          set(RED_DEFAULT_CODE);
          SmartDashboard.putBoolean("Red on", true);
        }
        break;
      case Blue:
        if(isShooting) {
          set(BLUE_SHOT_CODE);
        } else {
          set(BLUE_DEFAULT_CODE);
        }
      default:
        if(isShooting) {
          set(WHITE_SHOT_CODE);
        } else {
          set(WHITE_DEFAULT_CODE);
        }
    }
  }

 
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void setIsShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }
  
}