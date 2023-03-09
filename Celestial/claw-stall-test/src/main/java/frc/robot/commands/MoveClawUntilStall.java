// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Claw;

/** A command that will turn the robot to the specified angle. */
public class MoveClawUntilStall extends CommandBase {

  private Claw m_claw;
  double m_targetSpeed = 0.0;

  public MoveClawUntilStall(double speed, Claw claw) {
    addRequirements(claw);
    m_claw = claw;
    m_targetSpeed = speed;
  }

  @Override
  public void execute() {
    System.out.println("moving claw");
    m_claw.runClawClosedLoop(m_targetSpeed); // testing on celestial was with 10000 to 30000
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // finish when stalled, so we can transition to holding
    if (m_claw.checkStalledCondition()) {
      return true;
    } else {
      return false; // run this command only once, and it will run until stalled
    } 
  }

  @Override
  public WrapperCommand handleInterrupt(Runnable handler) {
    System.out.println("Hold claw being interrupted");
    return super.handleInterrupt(handler);
  }

}
