package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.HomeUpperArmCommand;
import frc.robot.commands.MoveUpperArmCommand;
import frc.robot.subsystems.UpperArmSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final UpperArmSubsystem m_upperArm = new UpperArmSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kY.value)
        .toggleOnTrue(new InstantCommand(
            () -> m_upperArm.nudgeClosedLoopByFalconEnc(true), m_upperArm ));

    new JoystickButton(m_driverController, Button.kA.value)
    .toggleOnTrue(new InstantCommand(
        () -> m_upperArm.nudgeClosedLoopByFalconEnc(false), m_upperArm ));

    new JoystickButton(m_driverController, Button.kB.value)
    .toggleOnTrue(new HomeUpperArmCommand(m_upperArm));
    
    new JoystickButton(m_driverController, Button.kX.value)
    .toggleOnTrue(new MoveUpperArmCommand(200, m_upperArm));

    
  }
}
