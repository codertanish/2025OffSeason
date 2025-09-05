package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO turret;
  private final TurretIOInputsAutoLogged tInputs = new TurretIOInputsAutoLogged();

  private static LoggedTunableNumber kV = new LoggedTunableNumber("CoralScoringArm/kV");

  private static LoggedTunableNumber kA = new LoggedTunableNumber("CoralScoringArm/kA", 0);

  private static LoggedTunableNumber kS = new LoggedTunableNumber("CoralScoringArm/kS", 0);

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private TrapezoidProfile turretProfile;
  private TrapezoidProfile.Constraints turretConstraints;

  private TrapezoidProfile.State armGoalStateDegrees = new TrapezoidProfile.State();
  private TrapezoidProfile.State armCurrentStateDegrees = new TrapezoidProfile.State();

  double goalDegrees;

  private SimpleMotorFeedforward turretFFModel;

  /** Creates a new Arm. */
  public Turret(TurretIO turret) {
    this.turret = turret;
    switch (SimConstants.currentMode) {
      case REAL:
        kV.initDefault(1);
        kA.initDefault(0);
        kS.initDefault(0);
        break;
      case REPLAY:
        kV.initDefault(1);
        kA.initDefault(1);
        kS.initDefault(1);
        break;
      case SIM:
        kV.initDefault(0.01);
        kA.initDefault(0);
        kS.initDefault(0);
        break;
      default:
        kV.initDefault(1);
        kA.initDefault(1);
        kS.initDefault(1);
        break;
    }

    // TODO: These are just the scoral arm values; these need to be changed to match
    // the actual mechanism.
    maxVelocityDegPerSec = 150;
    maxAccelerationDegPerSecSquared = 300;

    turretConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    turretProfile = new TrapezoidProfile(turretConstraints);

    armCurrentStateDegrees =
        turretProfile.calculate(0, armCurrentStateDegrees, armGoalStateDegrees);
    turretFFModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());

    updateTunableNumbers();
  }

  public void setBrakeMode(boolean bool) {
    turret.setBrakeMode(bool);
  }

  public double getArmPositionDegs() {
    return tInputs.positionDegs;
  }

  public boolean atGoal(double threshold) {
    return (Math.abs(tInputs.positionDegs - goalDegrees) <= threshold);
  }

  public boolean hasReachedGoal(double goalDegs) {
    return (Math.abs(tInputs.positionDegs - goalDegs) <= 8);
  }

  public void setConstraints(
      double maxVelocityDegreesPerSec, double maxAccelerationDegreesPerSecSquared) {
    turretConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityDegreesPerSec, maxAccelerationDegreesPerSecSquared);
    turretProfile = new TrapezoidProfile(turretConstraints);
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    // positionDegs = MathUtil.clamp(positionDegs, 33, 120);
    double currentVelRadPerSec = Math.toRadians(velocityDegsPerSec);
    Logger.recordOutput(
        "Turret Feedforward Volts",
        turretFFModel.calculateWithVelocities(currentVelRadPerSec, currentVelRadPerSec));
    turret.setPositionSetpointDegs(
        positionDegs,
        turretFFModel.calculateWithVelocities(
            currentVelRadPerSec,
            currentVelRadPerSec)); // TODO: Need to incorporate some function to calculate next
    // velocity in this.
  }

  public void setVolts(double volts) {
    turret.setVoltage(volts);
  }

  public void armStop() {
    turret.stop();
  }

  public void setArmGoal(double goalDegrees) {
    this.goalDegrees = goalDegrees;
    armGoalStateDegrees = new TrapezoidProfile.State(goalDegrees, 0);
  }

  public void setArmCurrent(double currentDegrees) {
    armCurrentStateDegrees = new TrapezoidProfile.State(currentDegrees, 0);
  }

  @Override
  public void periodic() {
    turret.updateInputs(tInputs);

    armCurrentStateDegrees =
        turretProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, armCurrentStateDegrees, armGoalStateDegrees);

    setPositionDegs(armCurrentStateDegrees.position, armCurrentStateDegrees.velocity);

    Logger.processInputs("Turret", tInputs);

    updateTunableNumbers();
  }

  private void updateTunableNumbers() {
    /* TODO: Implement this for tuning if needed:
      if (kV.hasChanged(hashCode()) || kA.hasChanged(hashCode()) || kS.hasChanged(hashCode()) {
        turretFFModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());

    }



    */
  }
}
