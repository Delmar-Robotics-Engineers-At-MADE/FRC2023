// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class StrafeToLimelight extends PIDCommand {
  
  private static PIDController m_PID = new PIDController(
    DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

  private static boolean m_shuffleboardLoaded = false;
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public 
  StrafeToLimelight(LimelightSubsystem limelight, DriveSubsystem drive) {
    super(
        m_PID,
        // Close loop on heading
        limelight::getLimelightBestWidth,
        // Set reference to target
        0, 
        // Pipe output to turn robot
        output -> drive.arcadeDrive(output, 0),
        // Require the drive
        drive);

        m_PID.setTolerance(0.5);
      
        // Add the PID to dashboard
      if (!m_shuffleboardLoaded) {
        ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
        turnTab.add("Limelight PID", m_PID);
        m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
      }
      System.out.println("new turn to limelight command created");
  
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
