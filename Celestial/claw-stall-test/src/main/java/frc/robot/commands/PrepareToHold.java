// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.CLAW_CONSTANTS;
import frc.robot.subsystems.Claw;

/** A command that will turn the robot to the specified angle. */
public class PrepareToHold extends CommandBase {

  private Claw m_claw;

  public PrepareToHold(Claw claw) {
    m_claw = claw;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    m_claw.prepareToHold();
    super.execute();
  }

  @Override
  public boolean isFinished() {
      return true;  
  }

  @Override
  public WrapperCommand handleInterrupt(Runnable handler) {
    System.out.println("Hold claw being interrupted");
    return super.handleInterrupt(handler);
  }

}