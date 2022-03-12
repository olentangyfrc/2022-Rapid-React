// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/*
*************************** 
Things need to Updated:
- Gyro Angle
- Location of Camera on Bot
- Send Position
***************************
*/

package frc.robot.subsystems;

import java.util.logging.Logger;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;




public class networkTables extends SubsystemBase {

  private SwerveDrivePoseEstimator poseEstimator;
  private Logger logger = Logger.getLogger("networkTables");

  NetworkTableInstance inst;
  Gyro gyro;
  NetworkTableEntry Time, XEntry, YEntry, ZEntry;

  
  public networkTables() {
    poseEstimator = SubsystemFactory.getInstance().getDrivetrain().getSwerveDrivePoseEstimator(); 
    inst = NetworkTableInstance.getDefault();
    gyro = SubsystemFactory.getInstance().getTelemetry().getGyro();
    inst.startClientTeam(4611);
    NetworkTable Tvecs = inst.getTable("SmartDashboard"); //delcares the networktables to the already intizialized instance

    Time = Tvecs.getEntry("Time");
    XEntry = Tvecs.getEntry("X");
    YEntry = Tvecs.getEntry("Y");
    ZEntry = Tvecs.getEntry("Z");

    Time.addListener(event -> {onVisionUpdate(event);}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
   
  }

  public void onVisionUpdate(EntryNotification event ){
    
    //System.out.println("Time changed value: " + event.getEntry().getValue()); //If time is changed, then it records x,y,z
    double time = event.getEntry().getDouble(0);
    //System.out.println("X changed value: " + XEntry.getValue());
    double x = XEntry.getDouble(0);
    //System.out.println("Y changed value: " + YEntry.getValue());
    double y = YEntry.getDouble(0);
    //System.out.println("Z changed value: " + ZEntry.getValue());
    double z = ZEntry.getDouble(0);

    var position = convertTvec(x, y, z); //converts the x,y,z into final position vector
    // Do work here like updates odometry...
    addVision(position.get(0, 0), position.get(1, 0), time);
    //System.out.print(position);


  }

 
  /**
   * 
   * @param x, @param y, @param z takes in the Tvec from Nano and converst between coordinate system
   * @return a vector that describes the robots position
   */
  public Matrix<N3, N1> convertTvec(double x, double y, double z) {
    // Build out input vector
    var in_vec = VecBuilder.fill(x, y, z);

    var cv2_correction_mat = Matrix.mat(Nat.N3(), Nat.N3()).fill(
      0, 0, 1,
      -1, 0, 0,
      0, -1, 0);

    var corrected_vec = cv2_correction_mat.times(in_vec);
    // Build our rotation matrix
    var pitch = -30 * Math.PI / 180;
    var c = Math.cos(pitch);
    var s = Math.sin(pitch);
    var camera_to_bot = Matrix.mat(Nat.N3(), Nat.N3()).fill(
       c, 0, s,
       0, 1, 0,
      -s, 0, c);

    // Rotate
    var corrected_bot_oriented = camera_to_bot.times(corrected_vec);


    var trans2_vec = VecBuilder.fill(0.3048, 0.1524, 0.9906); //Change this where we know the displacement of the camera to the center of the robot

    var out_vec = trans2_vec.plus(corrected_bot_oriented);
    double gyro_angle = gyro.getAngle() * Math.PI / 180; //add the heading by multipling by gyro angle
    
    

    c = Math.cos(gyro_angle);
    s = Math.sin(gyro_angle);
    var bot_to_field = Matrix.mat(Nat.N3(), Nat.N3()).fill(
       c, -s, 0,
       s, c, 0,
       0, 0, 1);
    var final_vec = bot_to_field.times(out_vec);
    
    var hub_coordinates = VecBuilder.fill( (16.46)/2, (8.23/2), 2.64);

    var position_vec =  hub_coordinates.minus(final_vec);

    // Print output


    SmartDashboard.putNumber("position_vecx", position_vec.get(0, 0));
    SmartDashboard.putNumber("position_vecy", position_vec.get(1, 0));
    SmartDashboard.putNumber("position_vecz", position_vec.get(2, 0)); 

    return position_vec; 
  }

  /**
       * Corrects Odermetry with Vision
       * 
       * @param x and @param y is Tvec from Nano
       * 
       * @param timeStampSeconds is when the vision measurement was made
       */
  public void addVision(double x, double y, double elapsedtime){
    poseEstimator.addVisionMeasurement(new Pose2d(x,y, gyro.getRotation2d()), Timer.getFPGATimestamp() - elapsedtime);
    /*
    try{
      poseEstimator.addVisionMeasurement(new Pose2d(x,y, gyro.getRotation2d()), timestampSeconds);
    }catch(Exception e){
      
    }
    */

}
  @Override
  public void simulationPeriodic() {
    periodic();
  }


}
