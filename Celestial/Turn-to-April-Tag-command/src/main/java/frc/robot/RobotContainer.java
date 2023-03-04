// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import static edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.TurnToAprilTag;
import frc.robot.commands.TurnToAprilTagProfiled;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.commands.UpdateBestAprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveToAprilTagProfiled;
import frc.robot.commands.RaiseHood;
import frc.robot.commands.RaiseWithPotentiometer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();


  // The driver's controller
  //PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getLeftY(), -m_driverController.getRightX()),
            m_robotDrive));

    m_hood.setDefaultCommand(new RunCommand (
      () -> m_hood.elevateWithGamepad(m_driverController.getLeftTriggerAxis(),
                           m_driverController.getRightTriggerAxis() ),
      m_hood
      ));
  }

  public void zeroHeading() {
    m_robotDrive.zeroHeading();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController,Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.kSlowSpeedFactor)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.kNormalSpeedFactor)));

    // Stabilize robot to drive straight with gyro when left bumper is held
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kStabilizationP,
                    DriveConstants.kStabilizationI,
                    DriveConstants.kStabilizationD),
                // Close the loop on the turn rate
                m_robotDrive::getTurnRate,
                // Setpoint is 0
                0,
                // Pipe the output to the turning controls
                output -> m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), output),
                // Require the robot drive
                m_robotDrive));

    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RepeatCommand( new TurnToAprilTagProfiled(0, m_robotDrive).withTimeout(5)));

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kB.value)
    .whileTrue(new RepeatCommand( new DriveToAprilTagProfiled(DriveConstants.GOAL_RANGE_METERS, m_robotDrive).withTimeout(5)));

    // Raise Hood when the 'Y' button is pressed
    // new JoystickButton(m_driverController, Button.kY.value)
    //     .onTrue(new RaiseHood(-150, m_hood));
    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new RaiseWithPotentiometer(0.045, m_hood));

    // Lower Hood when 'A' button is pressed
    // new JoystickButton(m_driverController, Button.kA.value)
    //    .onTrue(new RaiseHood(-25, m_hood));       
    new JoystickButton(m_driverController, Button.kA.value)
       .onTrue(new RaiseWithPotentiometer(.041, m_hood));       

    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .whileTrue(new RepeatCommand(new UpdateBestAprilTag(m_robotDrive)));
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