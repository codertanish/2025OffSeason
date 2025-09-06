package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.SubsystemConstants;

public class TurretIOSim implements TurretIO {
  // CHANGE THESE VALUES TO MATCH YOUR MOTOR AND GEARBOX
  private int gearBoxMotorCount = 1;
  private double gearing = SubsystemConstants.TurretConstants.gearRatio;
  private DCMotor motor = DCMotor.getKrakenX60Foc(gearBoxMotorCount);

  private DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.004, gearing), motor);

  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  private double clampedValueLowVolts = -12.0;
  private double clampedValueHighVolts = 12.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts,
              clampedValueLowVolts,
              clampedValueHighVolts);
      sim.setInputVoltage(appliedVolts);
    }
    // sim.setInputVoltage(appliedVolts);
    sim.update(SubsystemConstants.LOOP_PERIOD_SECONDS);

    inputs.positionDegs = 0;
    // inputs.velocityRadPerSec =
    // Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    inputs.velocityDegsPerSec = Units.radiansToDegrees(sim.getAngularVelocityRPM());
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
