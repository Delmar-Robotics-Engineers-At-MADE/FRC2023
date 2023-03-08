// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 24;
    public static final int kRightMotor2Port = 13;

    public static final double kNormalSpeedFactor = 0.2;
    public static final double kSlowSpeedFactor = 0.1;

    // public static final int[] kLeftEncoderPorts = new int[] {2,1};
    // public static final int[] kRightEncoderPorts = new int[] {24,13};
    // public static final boolean kLeftEncoderReversed = false;
    // public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = true;

    public static final double kStabilizationP = 0.8;
    public static final double kStabilizationI = 0.8;
    public static final double kStabilizationD = 0;

    public static final double kTurnP = 0.06;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kMaxTurnRateDegPerS = 25;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 1;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    // Constants such as camera and target height stored. Change per robot and goal!
    public final static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(36.5);
    public final static double TARGET_HEIGHT_METERS = Units.feetToMeters(3);
    
    // Angle between horizontal and the camera.
    public final static double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    public final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveToleranceDist = 1;
    //public static final double kDriveRateTolerance = 10; // degrees per second

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class HoodConstants {
    public static final int kHoodEncoderLimitLow = -20;
    public static final int kHoodEncoderLimitHigh = -200;
    public static final double kHoodP = 0.02;
    public static final double kHoodI = 0;
    public static final double kHoodD = 0;
    public static final double kDistanceTolerance = 1;
   ;
  }

  public static final class UpperArmConstants {

    public static final int UPPER_ARM_MOTOR_ID = 0;

    public static final int kRevEncoderLimitLow = -20;
    public static final int kRevEncoderLimitHigh = -200;
    public static final double kRevEncoderP = 0.003;
    public static final double kRevEncoderI = 0;
    public static final double kRevEncoderD = 0.002;
    public static final double kRevEncoderTolerance = 20; 
    public static final double kRevEncoderMaxCountsPerS = 750;
    public static final double kRevEncoderMaxCountsPerSSquared = 1000;
    public static final double kRevEncoderMaxCountsPerSTolerance = 100;

    public static final double kPotmeterP = 20;
    public static final double kPotmeterI = 2;
    public static final double kPotmeterD = 0;
    public static final double kPotmeterTolerance = 0.001;  

    public static final double kHomePotmeterValue = 0.54;
    public static final double kHomeEncoderValue = 0.0;

    public static final double kMaxFalconPower = 0.20;

    // for testing only
    public static final double kFalconClosedLoopTolerance = 100;
    public static final double kFalconTestNudgeAmount = 2000;
    public static final double kFalconP = 0.1;

  }

  public static final class CLAW_CONSTANTS {
    public static final int CLAW_ID = 6;
    public static final long kInVelocity = 10000;
    public static final long kConeOutVelocity = 2048;
    public static final long kCubeOutVelocity = 1228;
    public static final long kShootVelocity = 4096;
    public static final long kStopVelocity = 0;
    public static final long kHoldVelocity = 300;

    public static int kTimeoutMs;
    public static int kPIDLoopIdx;
    //                                                    kP     kI    kD  kF             Iz     PeakOut 
    public final static Gains kGains_Velocity = new Gains( 0.25, 0.0, 0.0, 0.08,  300,  1.00);
    // versa with encoder next to output shaft: 
    // public final static Gains kGains_Velocit = new Gains( 0.05, 0.0, 0.0, 0.24,  300,  1.00);
  }

  public static final class LowerArmConstants {

    public static final int LOWER_ARM_MOTOR_ID = 7;//24;
    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kFF = 0.0;
    public static double kMinOutput = -1;
    public static double kMaxOutput = 1;

    public static final double kHomePosition = 0;
    public static final double kFloorPosition = 0;
    public static final double kHighPosition = 0;
    public static final double kShootPosition = 0;
    public static final double kMidPosition = 0;
    public static final double kSSsPosition = 0;
    public static final double kManualSpeed = 0.7;
    public static final double minVelocity = 0;
    public static final double maxAccel = 1500;
    public static final double maxVelocity = 4000;
    public static final double allowedErr = 0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;


    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

}