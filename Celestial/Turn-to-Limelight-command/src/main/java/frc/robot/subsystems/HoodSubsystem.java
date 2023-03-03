package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.Constants.HoodConstants;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class HoodSubsystem extends SubsystemBase {

    private final DigitalInput m_limitSwitch = new DigitalInput(4);

    // private final AnalogInput m_potentiometer = new AnalogInput(0);
    private final AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(0);

    private final WPI_TalonSRX m_elevatorMotor = new WPI_TalonSRX(11);

    private final Encoder m_encoder = new Encoder(0, 1, 
                        false, Encoder.EncodingType.k4X);

    private boolean m_encoderHomed = false;

    private ShuffleboardTab m_driveBaseTab;

    public HoodSubsystem () {
        m_elevatorMotor.setNeutralMode(NeutralMode.Brake);
        m_driveBaseTab = Shuffleboard.getTab("Hood");
        m_driveBaseTab.add("Limit Switch", m_limitSwitch);
        m_driveBaseTab.add("Potentiometer2", m_potentiometer);
        m_driveBaseTab.addDouble("Pot2", () -> m_potentiometer.get());
        m_driveBaseTab.add("Encoder", m_encoder);
        m_driveBaseTab.addBoolean("Homed", () -> m_encoderHomed);
        // put homed widget on main dashboard tab, because not sure how to put it on tab using putBoolean
        // SmartDashboard.putBoolean("Encoder Homed", m_encoderHomed);
    }

    public double encoderPosition() {
        return m_encoder.getDistance();
    }

    public double potPosition() {
        return m_potentiometer.get();
    }

    public void elevateWithGamepad(double upSpeed, double downSpeed) {
        if (upSpeed > 0.0) {
            elevate(-upSpeed);
        } else {
            elevate (downSpeed);
        }
    }

    public void elevate(double speed) {
        // encoder goes negative as hood goes up
        if (m_encoderHomed && speed > 0 && m_encoder.getDistance() > HoodConstants.kHoodEncoderLimitLow) {
            // if homed, and we are close to limit switch, don't allow down
            m_elevatorMotor.set(0.0);
        } else if (m_encoderHomed && speed < 0 && m_encoder.getDistance() < HoodConstants.kHoodEncoderLimitHigh) {
            // if homed, don't allow hood to go up forever
            m_elevatorMotor.set(0.0);
        } else if (!m_encoderHomed && speed < 0) { 
            // not homed; don't allow up at all; operator should home first thing
            m_elevatorMotor.set(0.0);
        } else if (m_limitSwitch.get()) { // hood is NOT on limit switch; ok to go either way
            m_elevatorMotor.set(speed);
        } else if (!m_limitSwitch.get()){ // on limit switch; up is ok, but not down
            m_encoder.reset();
            m_encoderHomed = true;
            // SmartDashboard.putBoolean("Encoder Homed", m_encoderHomed);
            if (speed < 0) { // up is ok
                m_elevatorMotor.set(speed);
            } else { // down not ok
                m_elevatorMotor.set(0.0);
            }
        }
    }


}
