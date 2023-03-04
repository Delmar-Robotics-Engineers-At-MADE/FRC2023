package frc.robot.commands;

import frc.robot.subsystems.UpperArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NudgeArmWithFalconEnc extends CommandBase {

  private UpperArmSubsystem m_upperArm;

  public NudgeArmWithFalconEnc(UpperArmSubsystem upperArm) {
    m_upperArm = upperArm;
    double nudgeEncCounts = upperArm.m_nudgeDashboardEntry.getDouble(0);
    // double oldPosition = upperArm.getFalconEncPosition();
    System.out.println("move Falcon to " + (nudgeEncCounts));
    // upperArm.nudgeClosedLoopByFalconEnc(nudgeEncCounts);
  }

  @Override
  public boolean isFinished() {
    return true; // m_upperArm.isFalconAtSetpoint();
  }

  // @Override
  // public void execute() {
  //   double nudgeEncCounts = m_upperArm.m_nudgeDashboardEntry.getDouble(0);
  // }
}
