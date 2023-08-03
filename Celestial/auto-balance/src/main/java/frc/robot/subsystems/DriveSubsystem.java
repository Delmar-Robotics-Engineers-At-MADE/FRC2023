// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;



public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftFront = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_leftRear = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rightFront = new CANSparkMax(24, MotorType.kBrushless);
  private final CANSparkMax m_rightRear = new CANSparkMax(13, MotorType.kBrushless);

  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_leftFront, m_leftRear);
          // new CANSparkMax(DriveConstants.kLeftMotor1Port,MotorType.kBrushless),
          // new CANSparkMax(DriveConstants.kLeftMotor2Port,MotorType.kBrushless)
          // );

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightFront, m_rightRear);
          // new CANSparkMax(DriveConstants.kRightMotor1Port,MotorType.kBrushless),
          // new CANSparkMax(DriveConstants.kRightMotor2Port,MotorType.kBrushless)
          // );

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // // The left-side drive encoder
  // private final Encoder m_leftEncoder =
  //     new Encoder(
  //         DriveConstants.kLeftEncoderPorts[0],
  //         DriveConstants.kLeftEncoderPorts[1],
  //         DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         DriveConstants.kRightEncoderPorts[0],
  //         DriveConstants.kRightEncoderPorts[1],
  //         DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  //private final Gyro m_gyro = new ADXRS450_Gyro();
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

private double m_bestAprilTagYaw = 0.0;
private int m_bestAprilTagID = 0;
private double m_bestAprilTagPitch=0.0;
private double  m_bestAprilTagDistance = 0.0;

  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotors.setInverted(true);
    m_leftFront.setInverted(true);
    m_leftRear.setInverted(true);
    m_rightFront.setInverted(false);
    m_rightRear.setInverted(false);

    // zero power mode: break
    m_leftFront.setIdleMode(IdleMode.kBrake);
    m_leftRear.setIdleMode(IdleMode.kBrake);
    m_rightFront.setIdleMode(IdleMode.kBrake);
    m_rightRear.setIdleMode(IdleMode.kBrake);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    // Add the tank drive and encoders to dashboard
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Diff Drive", m_drive);
    // Add the gyro
    driveBaseTab.add("Gyro", m_gyro);
    driveBaseTab.addDouble("Gyro Disp X", () -> m_gyro.getDisplacementX());
    driveBaseTab.addDouble("Gyro Disp Y", () -> m_gyro.getDisplacementY());
    driveBaseTab.addDouble("Gyro Disp Z", () -> m_gyro.getDisplacementZ());
    // Put both encoders in a list layout
    // ShuffleboardLayout encoders =
    //     driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0, 0).withSize(2, 2);
    // encoders.add("Left Encoder", m_leftEncoder);
    // encoders.add("Right Encoder", m_rightEncoder);
    driveBaseTab.addDouble("Tag Yaw", () -> m_bestAprilTagYaw);
    driveBaseTab.addDouble("Tag ID", () -> m_bestAprilTagID);
    driveBaseTab.addDouble("Tag Distance", () -> m_bestAprilTagDistance);
    setMaxOutput(DriveConstants.kNormalSpeedFactor);

    // setup photon vision
    
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  //   m_leftEncoder.reset();
  //   m_rightEncoder.reset();
  // }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  // public double getAverageEncoderDistance() {
  //   return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  // }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  // public Encoder getLeftEncoder() {
  //   return m_leftEncoder;
  // }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  // public Encoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyro.resetDisplacement();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getRoll() {
    return Math.IEEEremainder(m_gyro.getRoll(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getDisplacementX() {
    return m_gyro.getDisplacementX();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
      
}






