package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.NudgeArmWithFalconEnc;
import frc.robot.subsystems.UpperArmSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final UpperArmSubsystem m_upperArm = new UpperArmSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kY.value)
            // .onTrue(new NudgeArmWithFalconEnc(m_upperArm));

        .toggleOnTrue(new InstantCommand(
            () -> m_upperArm.nudgeClosedLoopByFalconEnc(), m_upperArm ));
  }
}
