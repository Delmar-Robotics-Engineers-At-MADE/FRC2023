package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LowerArm;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final LowerArm m_lowerArm = new LowerArm();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();
    // m_claw.setDefaultCommand(new RunCommand(() -> m_claw.stop(), m_claw));
    // CommandScheduler.getInstance().setDefaultCommand(m_lowerArm, m_lowerArm.lowerArmHoldPosition());
    CommandScheduler.getInstance().setDefaultCommand(m_lowerArm, new RunCommand(() -> m_lowerArm.stop(), m_lowerArm));
  }

  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kY.value)
    .whileTrue(m_lowerArm.runLowerArmUp())
    .onFalse(m_lowerArm.stop());

    new JoystickButton(m_driverController, Button.kA.value)
    .whileTrue(m_lowerArm.runLowerArmDown())
    .onFalse(m_lowerArm.stop());



    
  }
}
