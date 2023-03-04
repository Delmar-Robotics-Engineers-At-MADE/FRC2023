package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.UpperArmConstants;


public class UpperArmSubsystem extends SubsystemBase {

    private final WPI_TalonFX m_upperArmMotor = new WPI_TalonFX(UpperArmConstants.UPPER_ARM_MOTOR_ID);
    private final AnalogPotentiometer m_potmeter = new AnalogPotentiometer(0);
    private final Encoder m_encoder = new Encoder(0, 1, 
                        false, Encoder.EncodingType.k4X);
    public boolean m_encoderHomed = false;
    private ShuffleboardTab m_armTab;
    public GenericEntry m_nudgeDashboardEntry;
    private GenericEntry m_falconPEntry;

    // Constructor
    public UpperArmSubsystem() {
        m_upperArmMotor.setNeutralMode(NeutralMode.Brake);

        // planning to use open-loop control on Falcon, plus our own PID controller using Rev encoder, so don't need PID settings

        // m_upperArmMotor.Config_kF(0, kFtuned, 30); // 2022 shooter Falcons were 30
        // m_upperArmMotor.Config_kP(0, kPtuned, 30);
        // m_upperArmMotor.Config_kI(0, 0.0, 30); // was .00005 in 2020
        // m_upperArmMotor.Config_kD(0, kDtuned, 30); // was 0 in 2020
        // m_upperArmMotor.ConfigClosedloopRamp(kRampTuned);
        // m_upperArmMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 30);

        ErrorCode err = m_upperArmMotor.configFactoryDefault(); if (err.value != 0) {System.out.println("Falcon config err: " + err.value);}
        err = m_upperArmMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30); if (err.value != 0) {System.out.println("Falcon config err: " + err.value);}
        err = m_upperArmMotor.setSelectedSensorPosition(0.0) ; if (err.value != 0) {System.out.println("Falcon config err: " + err.value);}

        m_armTab = Shuffleboard.getTab("Arm");
        m_armTab.addDouble("Shoulder Pot", () -> m_potmeter.get());
        m_armTab.add("Shoulder Enc", m_encoder);
        m_armTab.addBoolean("Homed", () -> m_encoderHomed);
        m_armTab.addDouble("Falcon Enc", () -> m_upperArmMotor.getSelectedSensorPosition());
        m_armTab.addDouble("Falcon Error", () -> m_upperArmMotor.getClosedLoopError());
        m_nudgeDashboardEntry = m_armTab.add("Nudge Amount", Constants.UpperArmConstants.kFalconTestNudgeAmount).getEntry();
        m_falconPEntry = m_armTab.add("Falcon P", Constants.UpperArmConstants.kFalconP).getEntry();
    }

    public double encoderPosition() {
        return m_encoder.getDistance();
    }

    public double potPosition() {
        return m_potmeter.get();
    }

    public void moveOpenLoop (double powerPercent) {
        m_upperArmMotor.set(ControlMode.PercentOutput, powerPercent);
        System.out.println("Falcon power: " + powerPercent);
    }

    public void setEncoderHomed() {
        m_encoderHomed = true;
        m_encoder.reset();
    }

    public void nudgeClosedLoopByFalconEnc () {
        double targetEncCounts = m_nudgeDashboardEntry.getDouble(0);
        double kP = m_falconPEntry.getDouble(0);
        ErrorCode err = m_upperArmMotor.config_kP(0, kP, 30); if (err.value != 0) {System.out.println("Falcon config err: " + err.value);}
        double currentPosition = m_upperArmMotor.getSelectedSensorPosition();
        System.out.println("setting target enc position to " + currentPosition + " + " + targetEncCounts);
        m_upperArmMotor.set(ControlMode.Position, currentPosition + targetEncCounts);
    }

    public double getFalconEncPosition () {
        return m_upperArmMotor.getSelectedSensorPosition();
    }

    public boolean isFalconAtSetpoint() {
        boolean result =  Math.abs(m_upperArmMotor.getClosedLoopError() 
                        - Constants.UpperArmConstants.kFalconClosedLoopTolerance)
               <= Constants.UpperArmConstants.kFalconClosedLoopTolerance;
        // System.out.println("Falcon at setpoint: " + result);
        return result;
    }

}