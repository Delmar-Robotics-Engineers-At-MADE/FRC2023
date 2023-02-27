// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultLighting;
import frc.robot.commands.SignalCones;
import frc.robot.commands.SignalCubes;
import frc.robot.subsystems.BlinkinSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final BlinkinSubsystem m_lights = new BlinkinSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();
    // m_lights.setDefaultCommand(new DefaultLighting(m_lights));
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController,Button.kX.value)
        .onTrue(new SignalCones(m_lights))
        .onFalse(new DefaultLighting(m_lights));

    new JoystickButton(m_driverController,Button.kY.value)
        .onTrue(new SignalCubes(m_lights))
        .onFalse(new DefaultLighting(m_lights));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // no auto
    return new InstantCommand();
  }
}
