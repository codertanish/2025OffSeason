package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;


public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;
  private final CANcoder turretCAN;
  private double positionSetpointDegs;

  private double startAngleDegs;

  private StatusSignal<Angle> turretPositionRotations;
  private final StatusSignal<AngularVelocity> velocityDegsPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Current> supplyCurrentAmps;

  public TurretIOTalonFX(int leadID, int canCoderID) {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    // change?
    canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // OFFSET IS IN ROTATIONS
    // coderConfig.MagnetSensor.withMagnetOffset(Units.degreesToRotations(58));

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.CurrentLimits.StatorCurrentLimit = SubsystemConstants.TurretConstants.CURRENT_LIMIT;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = SubsystemConstants.TurretConstants.CURRENT_LIMIT_ENABLED;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    talonConfig.Feedback.SensorToMechanismRatio = 1;
    turretMotor = new TalonFX(leadID);
    turretCAN = new CANcoder(canCoderID);

    turretMotor.getConfigurator().apply(talonConfig);
    turretCAN.getConfigurator().apply(canConfig);

    if (turretCAN.isConnected()) {
      turretMotor.setPosition(
          (turretCAN.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(57 - 12))
              * SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    } else {
      turretMotor.setPosition(
          Units.degreesToRotations(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG)
              * SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    }

    turretPositionRotations = turretMotor.getPosition();
    velocityDegsPerSec = turretMotor.getVelocity();
    appliedVolts = turretMotor.getMotorVoltage();
    statorCurrentAmps = turretMotor.getStatorCurrent();
    supplyCurrentAmps = turretMotor.getSupplyCurrent();

    positionSetpointDegs = SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG;

    Logger.recordOutput("start angle", startAngleDegs);

    turretMotor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        turretPositionRotations,
        velocityDegsPerSec,
        appliedVolts,
        statorCurrentAmps,
        supplyCurrentAmps);

  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretPositionRotations,
        velocityDegsPerSec,
        appliedVolts,
        statorCurrentAmps,
        supplyCurrentAmps);
 
    inputs.positionDegs =
        Units.rotationsToDegrees(turretPositionRotations.getValueAsDouble())
            / SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO;

    inputs.velocityDegsPerSec = Units.rotationsToDegrees(velocityDegsPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.positionSetpointDegs = positionSetpointDegs;

    Logger.recordOutput(
        "Turret Angular Position: ",
        Units.rotationsToDegrees(
          turretCAN.getAbsolutePosition().getValueAsDouble())); //TODO: Ideally, you should have some offset constant when actually implementing this.
  }

  @Override
  public void setBrakeMode(boolean bool) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    if (bool) {
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    turretMotor.getConfigurator().apply(config);
  }

  @Override
  public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
    this.positionSetpointDegs = positionDegs;

    turretMotor.setControl(
        new PositionVoltage(
                Units.degreesToRotations(positionDegs)
                    * SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO)
            .withFeedForward(ffVolts)); // CHECK FOR STOW ANGLE (positionDegs - 59)
  }

  @Override
  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    this.positionSetpointDegs = turretPositionRotations.getValueAsDouble();
    turretMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    turretMotor.getConfigurator().apply(config);
  }
}
