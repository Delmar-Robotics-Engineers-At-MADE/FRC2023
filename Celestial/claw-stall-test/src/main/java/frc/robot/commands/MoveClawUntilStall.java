package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

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

}
