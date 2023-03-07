// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class LimelightSubsystem extends SubsystemBase {

  private double m_bestArea = 0;
  private double m_bestWidth = 0;
  private double m_bestYaw = 0;
  private double highX = 0.0; 
  private double highY = 0.0; 
  private double lowX = 0.0; 
  private double lowY = 0.0;

  private static NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private boolean m_limelightOn = false;
  
  /** Creates a new DriveSubsystem. */
  public LimelightSubsystem() {

    ShuffleboardTab cameraTab = Shuffleboard.getTab("Cameras");
    cameraTab.addDouble("Limelight Area", () -> m_bestArea);
    cameraTab.addDouble("Limelight Width", () -> m_bestWidth);
    cameraTab.addDouble("Limelight Yaw", () -> m_bestYaw);
    cameraTab.addBoolean("Limelight On", () -> m_limelightOn);
  }


  // // call this from PID command to turn robot to best target
  // public double getBestLimelightYaw() {
  //   updateBestLimelight();
  //   return m_bestLimelightYaw;
  // }

  // public double getBestLimelightDistance() {
  //   updateBestLimelight();
  //   return m_bestLimelightDistance;
  // }

  public double getLimelightBestWidth(){
    // assume command to do this is called in combination, was: updateBestLimelight();
    return m_bestWidth;
  }

  public void updateBestLimelight() {
    turnLightOnOrOff(true);
    double pipeline = m_limelightTable.getEntry("getpipe").getDouble(-1.0);

    // low target pipeline
    if (pipeline == 0) {
      //System.out.println("pipeline 0");
      double targetsSeen = m_limelightTable.getEntry("tv").getDouble(0.0);
      if (targetsSeen > 0) {
        lowX = m_limelightTable.getEntry("tx").getDouble(0.0);
        lowY = m_limelightTable.getEntry("ty").getDouble(0.0);
        //System.out.println("low X: " + lowX);
      } else {
        lowX = 0; lowY = 0;
      }
      // switch to high target pipeline
      m_limelightTable.getEntry("pipeline").setNumber(1);
    }

    // high target pipeline
    if (pipeline > 0) {
      //System.out.println("pipeline 1");
      double targetsSeen = m_limelightTable.getEntry("tv").getDouble(0.0);
      if (targetsSeen > 0) {
        highX = m_limelightTable.getEntry("tx").getDouble(0.0);
        highY = m_limelightTable.getEntry("ty").getDouble(0.0);
        //System.out.println("high X: " + highX);
      } else {
        highX = 0; highY = 0;
      }
      // switch to low target pipeline
      m_limelightTable.getEntry("pipeline").setNumber(0);
    }

    if (lowX != 0 && highX != 0) {
      m_bestYaw = (lowX + highX) / 2;
      m_bestArea = (highX - lowX) * (highY - lowY);
      m_bestWidth = highX - lowX;
    } else {
      m_bestYaw = 0;
      m_bestArea = 0;
      m_bestWidth = 0;
    }

  }

  public void turnLightOnOrOff (boolean turnOn) {
    boolean turnOff = !turnOn;
    boolean lightIsOff = !m_limelightOn;
    if (m_limelightOn && turnOff) {
      System.out.println("sending command to turn OFF light");
      m_limelightTable.getEntry("ledMode").setNumber(1.0); // LED off
      m_limelightOn = false;
    } else if (lightIsOff && turnOn) {
      System.out.println("sending command to turn ON light");
      m_limelightTable.getEntry("ledMode").setNumber(3.0); // LED on bright
      m_limelightOn = true;
    }
  }
    
  // public void updateTargetArea() {
  //   System.out.println("update target area");
  //   turnLightOnOrOff(true);
  //   double targetsSeen = m_limelightTable.getEntry("tv").getDouble(0.0);
  //   System.out.println("target seen: " + targetsSeen);
  //   if (targetsSeen > 0) {
  //     m_LimelightTargetArea = m_limelightTable.getEntry("ta").getDouble(0.0);
  //   } else {
  //     m_LimelightTargetArea = 0;
  //   }
  //   System.out.println("target area: " + m_LimelightTargetArea);
  // }
}






