// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
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

  private double m_bestLimelightYaw = 0.0;
  private double  m_bestLimelightDistance = 0.0;

  private static NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private boolean m_limelightOn;
  
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
    // Put both encoders in a list layout
    // ShuffleboardLayout encoders =
    //     driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0, 0).withSize(2, 2);
    // encoders.add("Left Encoder", m_leftEncoder);
    // encoders.add("Right Encoder", m_rightEncoder);
    driveBaseTab.addDouble("Limelight Yaw", () -> m_bestLimelightYaw);
    driveBaseTab.addDouble("Limelight Distance", () -> m_bestLimelightDistance);
    driveBaseTab.addBoolean("Limelight On", () -> m_limelightOn);
    setMaxOutput(DriveConstants.kNormalSpeedFactor);

    // setup photon vision
    
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

      // call this from PID command to turn robot to best target
    public double getBestLimelightYaw() {
      updateBestLimelight();
      return m_bestLimelightYaw;
    }
  
    public double getBestLimelightDistance() {
      updateBestLimelight();
      return m_bestLimelightDistance;
    }

    public void updateBestLimelight() {
      turnLightOnOrOff(true);
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double targetsSeen = m_limelightTable.getEntry("tv").getDouble(0.0);
      if (targetsSeen > 0) {
        m_bestLimelightYaw = m_limelightTable.getEntry("tx").getDouble(0.0);
        m_bestLimelightDistance = 0.0; // try calculating this using AprilTag util
      } else {
        m_bestLimelightYaw = 0.0;
        m_bestLimelightDistance = 0.0;
      }
    }

    public void turnLightOnOrOff (boolean turnOn) {
      boolean turnOff = !turnOn;
      boolean lightIsOff = !m_limelightOn;
      if (m_limelightOn && turnOff) {
        System.out.println("sending command to turn OFF light");
        m_limelightTable.getEntry("ledMode").setNumber(1.0); // LED off
        m_limelightOn = false;
      } else if (lightIsOff && turnOn) {
        System.out.println("sending command to turn ON light");
        m_limelightTable.getEntry("ledMode").setNumber(3.0); // LED on bright
        m_limelightOn = true;
      }
    }
    
}






