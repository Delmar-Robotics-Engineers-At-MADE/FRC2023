package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;



public class DriveSubsystem extends SubsystemBase {

  private final Talon m_leftFront = new Talon(1);
  private final Talon m_leftRear = new Talon(0);
  private final Talon m_rightFront = new Talon(3);
  private final Talon m_rightRear = new Talon(2);

  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_leftFront, m_leftRear);

  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightFront, m_rightRear);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  public DriveSubsystem() {
    m_leftFront.setInverted(true);
    m_leftRear.setInverted(true);
    m_rightFront.setInverted(false);
    m_rightRear.setInverted(false);

    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Diff Drive", m_drive);
    driveBaseTab.add("Gyro", m_gyro);
    driveBaseTab.addDouble("Gyro Roll", () -> m_gyro.getRoll());

    setMaxOutput(DriveConstants.kNormalSpeedFactor);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }


  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}






