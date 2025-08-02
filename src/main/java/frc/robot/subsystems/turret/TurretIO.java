package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double velocityDegsPerSec = 0;
    public double positionDegs = 0;
    public double statorCurrentAmps = 0;
    public double supplyCurrentAmps = 0;
    public double appliedVolts = 0;
    public double positionSetpointDegs;

    public boolean cancoderConnected = false;
    public double pitch = 0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setBrakeMode(boolean bool) {}

  public default void setPositionSetpointDegs(double positionDegs, double ffVolts) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
