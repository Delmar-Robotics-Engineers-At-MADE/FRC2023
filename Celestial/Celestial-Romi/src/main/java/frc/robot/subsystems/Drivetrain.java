// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.sensors.RomiGyro;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final CANSparkMax m_leftFront = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_leftRear = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rightFront = new CANSparkMax(24, MotorType.kBrushless);
  private final CANSparkMax m_rightRear = new CANSparkMax(13, MotorType.kBrushless);


  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final RelativeEncoder m_leftFrontEncoder = m_leftFront.getEncoder();
  private final RelativeEncoder m_rightFrontEncoder = m_leftFront.getEncoder();

  // Set up the differential drive controller
 private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftFront, m_leftRear);
 private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightFront, m_rightRear);
 private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_left, m_right);
 
  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightFront.setInverted(true);
    //m_rightRear.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftFrontEncoder.setPositionConversionFactor((Math.PI * kWheelDiameterInch) / kCountsPerRevolution * 12);
    m_rightFrontEncoder.setPositionConversionFactor((Math.PI * kWheelDiameterInch) / kCountsPerRevolution * 12);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftFrontEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
  }

  public double getLeftEncoderCount() {
    return m_leftFrontEncoder.getPosition();
  }

  public double getRightEncoderCount() {
    return m_rightFrontEncoder.getPosition();
  }

  public double getLeftDistanceInch() {
    return m_leftFrontEncoder.getPosition();
  }

  public double getRightDistanceInch() {
    return m_rightFrontEncoder.getPosition();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
