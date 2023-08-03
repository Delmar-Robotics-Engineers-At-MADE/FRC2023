package frc.robot.commands;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class ProfiledDoublePIDCommand extends CommandBase {

    public static class DoubleDouble {
        public double x1;
        public double x2;

        public DoubleDouble() {}

        public DoubleDouble(double first, double second) {
            this.x1 = first;
            this.x2 = second;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof DoubleDouble) {
                DoubleDouble rhs = (DoubleDouble) other;
                return this.x1 == rhs.x1 && this.x2 == rhs.x2;
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(x1, x2);
        }
    }

    public static class DoubleState {
        public State s1;
        public State s2;

        public DoubleState() {}

        public DoubleState(State first, State second) {
            this.s1 = first;
            this.s2 = second;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof DoubleState) {
                DoubleState rhs = (DoubleState) other;
                return this.s1.equals(rhs.s1) && this.s2.equals(rhs.s2);
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(s1.hashCode(), s2.hashCode());
        }
    }




    protected final ProfiledPIDController m_controller;
    protected DoubleSupplier m_measurement;
    protected Supplier<State> m_goal1;
    protected final ProfiledPIDController m_controller2;
    protected DoubleSupplier m_measurement2;
    protected Supplier<State> m_goal2;
    protected BiConsumer<DoubleDouble, DoubleState> m_useOutput;

  public ProfiledDoublePIDCommand(
    ProfiledPIDController controller,
    ProfiledPIDController controller2,
    DoubleSupplier measurementSource,
    DoubleSupplier measurementSource2,
    DoubleSupplier goalSource1,
    DoubleSupplier goalSource2,
    BiConsumer<DoubleDouble, DoubleState> useOutput,
      Subsystem... requirements) {
    requireNonNullParam(controller, "controller", "SynchronousPIDCommand");
    requireNonNullParam(measurementSource, "measurementSource", "SynchronousPIDCommand");
    requireNonNullParam(goalSource1, "goalSource", "SynchronousPIDCommand");
    requireNonNullParam(useOutput, "useOutput", "SynchronousPIDCommand");

    m_controller = controller;
    m_useOutput = useOutput;
    m_measurement = measurementSource;
    m_goal1 = () -> new State(goalSource1.getAsDouble(), 0);
    m_controller2 = controller2;
    m_measurement2 = measurementSource2;
    m_goal2 = () -> new State(goalSource2.getAsDouble(), 0);
    m_requirements.addAll(Set.of(requirements));
  }

  public ProfiledDoublePIDCommand(
    ProfiledPIDController controller, ProfiledPIDController controller2,
    DoubleSupplier measurementSource, DoubleSupplier measurementSource2,
    double goal1, double goal2,
      BiConsumer<DoubleDouble, DoubleState> useOutput,
      Subsystem... requirements) {
        this(controller, controller2, measurementSource, measurementSource2, 
             () -> goal1, () -> goal2, useOutput, requirements);
  }

  @Override
  public void initialize() {
    m_controller.reset(m_measurement.getAsDouble());
    m_controller2.reset(m_measurement2.getAsDouble());
  }

  @Override
  public void execute() {
    m_useOutput.accept(
        new DoubleDouble (m_controller.calculate(m_measurement.getAsDouble(), m_goal1.get()), 
                          m_controller2.calculate(m_measurement2.getAsDouble(), m_goal2.get())),
        new DoubleState(m_controller.getSetpoint(), m_controller2.getSetpoint()));
  }

  @Override
  public void end(boolean interrupted) {
    m_useOutput.accept(new DoubleDouble(0,0), new DoubleState(new State(), new State()));
  }

  public ProfiledPIDController getController1() {
    return m_controller;
  }
  public ProfiledPIDController getController2() {
    return m_controller2;
  }
}
