// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.commands.ProfiledDoublePIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class AutoBalanceCommand extends ProfiledDoublePIDCommand {
  
  // turn PID
  private static ProfiledPIDController m_PID1 = new ProfiledPIDController(
    DriveConstants.kStabilizationP, DriveConstants.kStabilizationI, DriveConstants.kStabilizationD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared));

  // balance PID
  private static ProfiledPIDController m_PID2 = new ProfiledPIDController(
    DriveConstants.kBalanceP, DriveConstants.kBalanceI, DriveConstants.kBalanceD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxBalanceRateDegPerS,
                DriveConstants.kMaxBalanceAccelerationDegPerSSquared));


  private static boolean m_shuffleboardLoaded = false;
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public AutoBalanceCommand(DriveSubsystem drive) {
    super(
        m_PID1, m_PID2,
        // rotate and balance
        drive::getHeading, drive::getBalance,
        // Set reference to target
        0, 0,
        // Pipe output to turn robot
        (output, setpoint) -> drive.arcadeDrive(
          Math.max(-1,Math.min(1,output.x2)), Math.max(-1,Math.min(0.85,output.x1))),
        // Require the drive
        drive);

    // controller1: rotate
    // controller2: strafe

    // Set the controller to be continuous (because it is an angle controller)
    getController1().enableContinuousInput(-180, 180);
    getController2().enableContinuousInput(-180, 180);

    getController1().setTolerance(DriveConstants.kYawToleranceDeg, DriveConstants.kYawRateToleranceDegPerS);
    getController2().setTolerance(DriveConstants.kBalanceToleranceDeg, DriveConstants.kBalanceRateToleranceDegPerS);

    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
      turnTab.add("Double PID 1", m_PID1);
      turnTab.add("Double PID 2", m_PID2);
      m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
    }

  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    boolean done = getController1().atGoal() && getController2().atGoal();
    if (done) {
      System.out.println("done balancing");
    }
    return done;
  }

  @Override
  public void execute() {
    // System.out.println("stafing to April Tag " + m_aprilTags.getBestAprilTag3dAngle());
    super.execute();
  }  
}
