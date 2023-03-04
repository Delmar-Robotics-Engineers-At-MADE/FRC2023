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
}
