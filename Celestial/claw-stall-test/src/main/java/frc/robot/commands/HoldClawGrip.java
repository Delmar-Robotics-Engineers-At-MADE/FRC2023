// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Claw;

/** A command that will turn the robot to the specified angle. */
public class HoldClawGrip extends CommandBase {

  private Claw m_claw;
  double m_targetPosition = 0.0;

  public HoldClawGrip(double position, Claw claw) {
    m_claw = claw;
    m_targetPosition = position;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    m_claw.hold(m_targetPosition);
    super.execute();
  }

  @Override
  public boolean isFinished() {
      return (m_claw.m_cancelHold);  // run command once and it will continue until interrupted
  }

  @Override
  public WrapperCommand handleInterrupt(Runnable handler) {
    System.out.println("Hold claw being interrupted");
    return super.handleInterrupt(handler);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelSelf;
  }
}
