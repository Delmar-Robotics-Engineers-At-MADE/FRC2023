package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.CLAW_CONSTANTS;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.HoldClawGrip;
import frc.robot.commands.MoveClawUntilStall;
import frc.robot.commands.PrepareToHold;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final Claw m_claw = new Claw();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final SequentialCommandGroup m_moveAndHoldCommand= new SequentialCommandGroup(
    new MoveClawUntilStall(CLAW_CONSTANTS.kInVelocity, m_claw), 
    new PrepareToHold(m_claw),
    new HoldClawGrip(0.0, m_claw)
  );

  public RobotContainer() {
    configureButtonBindings();
    CommandScheduler.getInstance().setDefaultCommand(m_claw, m_claw.stop());
  }

  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kA.value)
        .toggleOnTrue(m_moveAndHoldCommand);



    
  }
}
