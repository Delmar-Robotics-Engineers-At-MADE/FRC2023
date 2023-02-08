/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private int m_index = 0;
  private Joystick m_stick;
  private static final int deviceID[] = {1, 4, 11, 14, 8, 12, 7, 3};
  private CANSparkMax m_motor[];
  private SparkMaxPIDController m_pidController[];
  private RelativeEncoder m_encoder[];
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Motor Index", m_index);

    m_stick = new Joystick(0);

    // initialize motors

    m_motor = new CANSparkMax[8];
    m_pidController = new SparkMaxPIDController[8];
    m_encoder = new RelativeEncoder[8];

    for (int i=0; i<8; i++) {
      m_motor[i] = new CANSparkMax(deviceID[i], MotorType.kBrushless);
    }

    // PID coefficients
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    for (int i=0; i<8; i++) {
        /**
       * The RestoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */
      m_motor[i].restoreFactoryDefaults();

      /**
       * In order to use PID functionality for a controller, a SparkMaxPIDController object
       * is constructed by calling the getPIDController() method on an existing
       * CANSparkMax object
       */
      m_pidController[i] = m_motor[i].getPIDController();

      // Encoder object created to display position values
      m_encoder[i] = m_motor[i].getEncoder();

      // set PID coefficients
      m_pidController[i].setP(kP);
      m_pidController[i].setI(kI);
      m_pidController[i].setD(kD);
      m_pidController[i].setIZone(kIz);
      m_pidController[i].setFF(kFF);
      m_pidController[i].setOutputRange(kMinOutput, kMaxOutput);
    }

    // display the PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  @Override
  public void teleopPeriodic() {
    // which motor controller to work with
    m_index = (int)SmartDashboard.getNumber("Motor Index", 0);
    SmartDashboard.putNumber("Selected Motor", deviceID[m_index]);

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController[m_index].setP(p); kP = p; }
    if((i != kI)) { m_pidController[m_index].setI(i); kI = i; }
    if((d != kD)) { m_pidController[m_index].setD(d); kD = d; }
    if((iz != kIz)) { m_pidController[m_index].setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController[m_index].setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController[m_index].setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    double setPoint = m_stick.getY()*maxRPM;
    m_pidController[m_index].setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", m_encoder[m_index].getVelocity());
  }
}
