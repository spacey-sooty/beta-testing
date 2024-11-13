package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public class Shooter extends SubsystemBase {
  private static final int PORT = 99;
  private static final int SENSOR_CHANNEL = 30;
  private static final double P = 0.1;
  private static final double I = 0;
  private static final double D = 0;
  private static final double S = 0;
  private static final double V = 0;
  private static final double A = 0;

  private final SparkMax motor = new SparkMax(PORT, MotorType.kBrushless);
  private final SparkMaxSim motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
  private final FlywheelSim flywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 1, 1.25), DCMotor.getNEO(1));
  private final PIDController pid = new PIDController(P, I, D);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(S, V, A);
  private final DigitalInput noteSensor = new DigitalInput(SENSOR_CHANNEL);

  public final Trigger atSetpoint = new Trigger(pid::atSetpoint);
  public final Trigger hasNote = new Trigger(noteSensor::get);

  /**
   * @param velocity velocity in RPM
   */
  public Command shoot(double velocity) {
    return run(() -> {
      var pid_output = pid.calculate(motor.getEncoder().getVelocity(), velocity);
      var ff_output = ff.calculate(velocity);
      if (RobotBase.isSimulation()) {
        flywheelSim.setInputVoltage(pid_output + ff_output);
      }
      motor.setVoltage(pid_output + ff_output);
    });
  }

  public Command stop() {
    return run(() -> {
      if (RobotBase.isSimulation()) {
        flywheelSim.setInputVoltage(0);
      }
      motor.setVoltage(0);
    });
  }

  @Override
  public void simulationPeriodic() {
    flywheelSim.update(0.2);
    motorSim.iterate(flywheelSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
  }
}
