package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem controls the decorative LED lights around the shooter wheel of the robot
 */
public class Led_Lights extends SubsystemBase {

  // Used to send PWM signals to the LEDs
  private static Spark m_blinkin;

  // Color codes
  public static final double RED_DEFAULT_CODE = -0.85;
  public static final double BLUE_DEFAULT_CODE = -0.83;
  public static final double WHITE_DEFAULT_CODE = -0.81;

  public static final double RED_SHOT_CODE = 0.61;
  public static final double BLUE_SHOT_CODE = 0.87;
  public static final double WHITE_SHOT_CODE = 0.93;

  private boolean isShooting = false;
  
  public Led_Lights(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
  }

  @Override
  public void periodic() {
    switch(DriverStation.getAlliance()) {
      case Red:
        if(isShooting) {
          set(RED_SHOT_CODE);
        } else {
          set(RED_DEFAULT_CODE);
  
        }
        break;
      case Blue:
        if(isShooting) {
          set(BLUE_SHOT_CODE);
        } else {
          set(BLUE_DEFAULT_CODE);
        }
        
        break;

      default:
        if(isShooting) {
          set(WHITE_SHOT_CODE);
        } else {
          set(WHITE_DEFAULT_CODE);
        }
    }
  }

  /**
   * Set the given color code to the LEDs.
   * 
   * @param val The color code to set the LEDs to. If this isn't in the range [-1, 1] it will not be set.
   */
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  /**
   * Tell the LED subsystem whether or not the robot is shooting
   * 
   * @param isShooting True if the robot is shooting, otherwise, false
   */
  public void setIsShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }
  
}