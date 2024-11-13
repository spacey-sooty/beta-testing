package frc.robot.util;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {
  public SparkMaxLogger() {
    super(SparkMax.class);
  }

  @Override
  public void update(DataLogger dataLogger, SparkMax motor) {
    dataLogger.log("AngularVelocityRPM", motor.getEncoder().getVelocity());
    dataLogger.log("AngleRotations", motor.getEncoder().getPosition());
    dataLogger.log("VoltageVolts", motor.getBusVoltage() * motor.getAppliedOutput());
    dataLogger.log("CurrentAmps", motor.getOutputCurrent());
  }
}
