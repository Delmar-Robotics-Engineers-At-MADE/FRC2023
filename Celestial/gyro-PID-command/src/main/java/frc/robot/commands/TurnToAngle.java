// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends PIDCommand {

  private static PIDController m_PID = new PIDController(
    DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

  private static boolean m_shuffleboardLoaded = false;
  
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        m_PID,
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(0, output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
      turnTab.add("Gyro PID", m_PID);
      m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
    }
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    // System.out.println(getController().getPositionError());
    // boolean result = Math.abs(getController().getPositionError()) 
    //   <= DriveConstants.kTurnToleranceDeg;
    // return result;
    return getController().atSetpoint();
  }
}
