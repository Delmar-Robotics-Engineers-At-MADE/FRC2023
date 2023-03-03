// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will turn the robot to the specified angle. */
public class UpdateBestAprilTag extends CommandBase {

  private DriveSubsystem m_drive;

  public UpdateBestAprilTag(DriveSubsystem drive) {
    m_drive = drive;
  }

  @Override
  public void execute() {
    m_drive.updateBestAprilTag();
  }

  // @Override
  // public boolean isFinished() {
  //   return true; //  run only while button is held down
  // }
}
