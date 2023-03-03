// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will turn the robot to the specified angle. */
public class RaiseWithPotentiometer extends PIDCommand {

  private static PIDController m_PID = new PIDController(
    250,  HoodConstants.kHoodI,HoodConstants.kHoodD);

  private static boolean m_shuffleboardLoaded = false;

  /**
   * Raises hood to a specified distance.
   *
   * @param targetDistance The encoder distance to go to
   * @param hood The subsystem to use
   */
  public RaiseWithPotentiometer(double targetDistance, HoodSubsystem hood) {
    super(
        m_PID,
        // Close loop on hood position
        hood::potPosition,
        // Set reference to target distance
        targetDistance,
        // Pipe output to hood to elevate
        output -> hood.elevate(output),
        // Require the hood subsystem
        hood);

    // Set the controller to be continuous (because it is an angle controller)
    //getController().enableContinuousInput(-180, 180);
    
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
      getController().setTolerance(0.001);
      
    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab dashboardTab = Shuffleboard.getTab("Hood");
      dashboardTab.add("Raiser PID2", m_PID);
      m_shuffleboardLoaded = true; // so we do this only once for the class
    }
    System.out.println("move with pot to " + targetDistance);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    // System.out.println(getController().getPositionError());
    // boolean result = Math.abs(getController().getPositionError()) 
    //   <= DriveConstants.kTurnToleranceDeg;
    // return result;
    return getController().atSetpoint();
  }

  // @Override
  // public void execute() {
  //   System.out.println("move with pot " + m_PID.getPositionError() + " " + m_PID.getSetpoint() + m_PID.getPositionTolerance());
  // }
}
