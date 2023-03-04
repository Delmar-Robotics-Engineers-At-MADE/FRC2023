package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final Claw m_claw = new Claw();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();
    // m_claw.setDefaultCommand(new RunCommand(() -> m_claw.stop(), m_claw));
    CommandScheduler.getInstance().setDefaultCommand(m_claw, m_claw.stop());
  }

  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kA.value)
    .whileTrue(m_claw.in());



    
  }
}
