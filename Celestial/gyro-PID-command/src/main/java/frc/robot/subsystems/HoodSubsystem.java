package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class HoodSubsystem extends SubsystemBase {

    private final DigitalInput m_limitSwitch = new DigitalInput(4);

    private final WPI_TalonSRX m_elevatorMotor = new WPI_TalonSRX(11);

    private final Encoder m_encoder = new Encoder(0, 1, 
                        false, Encoder.EncodingType.k4X);

    private boolean m_encoderHomed = false;

    private ShuffleboardTab m_driveBaseTab;

    public HoodSubsystem () {
        m_elevatorMotor.setNeutralMode(NeutralMode.Brake);
        m_driveBaseTab = Shuffleboard.getTab("Hood");
        m_driveBaseTab.add("Limit Switch", m_limitSwitch);
        m_driveBaseTab.add("Encoder", m_encoder);
        m_driveBaseTab.add("Encoder Homed", m_encoderHomed);
    }

    public void elevate(double upSpeed, double downSpeed) {
        if (upSpeed > 0.0) {
            m_elevatorMotor.set(-upSpeed);
        } else if (m_limitSwitch.get()) {
            m_elevatorMotor.set(downSpeed);
        } else if (!m_limitSwitch.get()) { // on limit switch
            m_elevatorMotor.set(0.0);
            m_encoder.reset();
            m_encoderHomed = true;
            SmartDashboard.putBoolean("Encoder Homed", m_encoderHomed);
        } else {
            m_elevatorMotor.set(0.0);
        }
    }
}
