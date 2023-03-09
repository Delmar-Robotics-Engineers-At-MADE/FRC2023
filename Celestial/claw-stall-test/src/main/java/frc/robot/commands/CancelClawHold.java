package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Claw;

public class CancelClawHold extends CommandBase {

  private Claw m_claw;

  public CancelClawHold(Claw claw) {
    m_claw = claw;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    m_claw.cancelHold();
    super.execute();
  }

  @Override
  public boolean isFinished() {
      return true;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelSelf;
  }
}
