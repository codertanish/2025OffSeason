// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.*;
import frc.robot.constants.FieldConstants.Barge;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static double sidewaysError = 0;
  private static double forwardsError = 0;
  private static double rotationError = 0;

  private static double wantedSidewaysVelocity = 0;
  private static double wantedRotationVelocity = 0;
  private static double wantedForwardsVelocity = 0;

  private static double forwardsAssistEffort = 0;
  private static double sidewaysAssistEffort = 0;
  private static double rotationAssistEffort = 0;

  private static Pose2d previousTargetPose;
  private static Pose2d targetPose;

  public static Pose2d getTargetPose() {
    return targetPose;
  }

  // private static ProfiledPIDController goof = new ProfiledPIDController(1.5, 0, 0, )

  // profiled controllers

  static ProfiledPIDController sidewaysPID =
      new ProfiledPIDController(7, 1, 0.5, new TrapezoidProfile.Constraints(3, 4.5));
  static ProfiledPIDController forwardsPID =
      new ProfiledPIDController(7, 1, 0.5, new TrapezoidProfile.Constraints(3, 4.5));
  static ProfiledPIDController rotationPID =
      new ProfiledPIDController(2.9, 0., 0.2, new TrapezoidProfile.Constraints(120, 150));
  // new ProfiledPIDController(0, 0., 0, new TrapezoidProfile.Constraints(70, 120));

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      SuperStructure superStructure,
      LED led,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier angleAssistSupplier,
      BooleanSupplier reefLeftSupplier,
      BooleanSupplier reefRightSupplier) {
    // BooleanSupplier sourceAlignSupplier,
    // BooleanSupplier processorAlignSupplier,
    // BooleanSupplier anchorAlignSupplier) {
    return Commands.run(
        () -> {
          rotationPID.setTolerance(1);
          rotationPID.enableContinuousInput(-180, 180);
          sidewaysPID.setTolerance(0.1);
          forwardsPID.setTolerance(0.1);

          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);
          Logger.recordOutput("LinearVelocityX: ", linearVelocity.getX());
          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

          double forwardSpeed = speeds.vxMetersPerSecond;

          double sidewaysSpeed = speeds.vyMetersPerSecond;

          double rotationSpeed = speeds.omegaRadiansPerSecond;

          double speedDebuff = 0.75;

          targetPose = null;
          if (reefLeftSupplier.getAsBoolean() || reefRightSupplier.getAsBoolean()) {
            led.setState(LED_STATE.FLASHING_RED);
            Translation2d reefTranslation =
                drive.isNearReef() ? new Translation2d(-0.5, 0) : new Translation2d(-1.3, 0);

            if (reefLeftSupplier.getAsBoolean()) {
              targetPose = drive.getNearestCenterLeft();
              targetPose = rotateAndNudge(targetPose, reefTranslation, new Rotation2d(Math.PI));
            } else if (reefRightSupplier.getAsBoolean()) {
              targetPose = drive.getNearestCenterRight();
              targetPose = rotateAndNudge(targetPose, reefTranslation, new Rotation2d(Math.PI));
            } else {
              targetPose = drive.getNearestCenter();
              targetPose = rotateAndNudge(targetPose, reefTranslation, new Rotation2d(Math.PI));
            }
            Logger.recordOutput("drive targetPose name", "reef");

          } else if (angleAssistSupplier.getAsBoolean()
              && superStructure.getWantedState() == SuperStructureState.SOURCE) {
            targetPose = drive.getNearestSource();
            targetPose = rotateAndNudge(targetPose, new Translation2d(0.5, 0), new Rotation2d(0));

            Logger.recordOutput("drive targetPose name", "source");

          } else if (angleAssistSupplier.getAsBoolean()
              && superStructure.getWantedState() == SuperStructureState.PROCESSOR) {
            targetPose = FieldConstants.Processor.centerFace;
            targetPose =
                rotateAndNudge(targetPose, new Translation2d(-0.5, 0), new Rotation2d(Math.PI));
            speedDebuff *= 0.5;

            Logger.recordOutput("drive targetPose name", "processor");
          } else if (angleAssistSupplier.getAsBoolean()
              && superStructure.getWantedState() == SuperStructureState.CLIMB_STAGE_ONE) {
            targetPose =
                drive
                    .getPose()
                    .nearest(
                        new ArrayList<>(
                            Arrays.asList(Barge.closeCage, Barge.middleCage, Barge.farCage)));
            targetPose =
                rotateAndNudge(
                    new Pose2d(targetPose.getTranslation(), targetPose.getRotation()),
                    new Translation2d(-0.5, 0),
                    new Rotation2d(0));

            Logger.recordOutput("drive targetPose name", "anchor");
          } else {
            Logger.recordOutput("drive targetPose name", "none");
          }

          if (targetPose != null && !targetPose.equals(previousTargetPose)) {
            previousTargetPose = targetPose;
            sidewaysPID.reset(drive.getPose().getY());
            forwardsPID.reset(drive.getPose().getX());
            rotationPID.reset(drive.getRotation().getDegrees());
          }

          Logger.recordOutput("drive targetPose", targetPose);

          if (targetPose != null) {
            forwardsError = drive.getPose().getX() - targetPose.getX();
            sidewaysError = drive.getPose().getY() - targetPose.getY();

            // hacky code so that it always rotates the shorter direction
            double driveDegrees = drive.getPose().getRotation().getDegrees() % 360;
            if (driveDegrees < 0) {
              driveDegrees += 360;
            }
            double targetDegrees = targetPose.getRotation().getDegrees() % 360;
            if (targetDegrees < 0) {
              targetDegrees += 360;
            }
            if (driveDegrees - targetDegrees > 180) {
              driveDegrees -= 360;
            } else if (targetDegrees - driveDegrees > 180) {
              targetDegrees -= 360;
            }

            rotationError = driveDegrees - targetDegrees;

            wantedForwardsVelocity =
                MathUtil.clamp(
                    forwardsPID.calculate(drive.getPose().getX(), targetPose.getX()), -3, 3);
            wantedSidewaysVelocity =
                MathUtil.clamp(
                    sidewaysPID.calculate(drive.getPose().getY(), targetPose.getY()), -3, 3);
            wantedRotationVelocity =
                MathUtil.clamp(
                    Math.toRadians(
                        rotationPID.calculate(
                            drive.getRotation().getDegrees(),
                            targetPose.getRotation().getDegrees())),
                    -drive.getMaxAngularSpeedRadPerSec(),
                    drive.getMaxAngularSpeedRadPerSec());

            forwardsAssistEffort =
                reefLeftSupplier.getAsBoolean() || reefRightSupplier.getAsBoolean()
                    ? (wantedForwardsVelocity - forwardSpeed) * speedDebuff
                    : 0;
            sidewaysAssistEffort =
                reefLeftSupplier.getAsBoolean() || reefRightSupplier.getAsBoolean()
                    ? (wantedSidewaysVelocity - sidewaysSpeed) * speedDebuff
                    : 0;

            rotationAssistEffort = (wantedRotationVelocity - rotationSpeed) * speedDebuff;

          } else {
            wantedForwardsVelocity = forwardSpeed;
            forwardsAssistEffort = 0;

            wantedSidewaysVelocity = sidewaysSpeed;
            sidewaysAssistEffort = 0;
            wantedRotationVelocity = rotationSpeed;
            rotationAssistEffort = 0;
          }

          Logger.recordOutput("target pose", targetPose);

          Logger.recordOutput("Forwards Profile Position", forwardsPID.getSetpoint().position);
          Logger.recordOutput("Sideways Profile Position", sidewaysPID.getSetpoint().position);
          Logger.recordOutput("Rotation Profile Position", rotationPID.getSetpoint().position);

          Logger.recordOutput("Forwards Profile Velocity", forwardsPID.getSetpoint().velocity);
          Logger.recordOutput("Sideways Profile Velocity", sidewaysPID.getSetpoint().velocity);
          Logger.recordOutput("Rotation Profile Velocity", rotationPID.getSetpoint().velocity);

          Logger.recordOutput("Forwards Error", forwardsError);
          Logger.recordOutput("Sideways Error", sidewaysError);
          Logger.recordOutput("Rotation Error", rotationError);

          Logger.recordOutput("Wanted Sideways Velocity", wantedSidewaysVelocity);
          Logger.recordOutput("Wanted Forwards Velocity", wantedForwardsVelocity);
          Logger.recordOutput("Wanted Rotation Velocity", wantedRotationVelocity);

          Logger.recordOutput("Forwards Assist Effort", forwardsAssistEffort);
          Logger.recordOutput("Sideways Assist Effort", sidewaysAssistEffort);
          Logger.recordOutput("Rotation Assist Effort", rotationAssistEffort);

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  new ChassisSpeeds(
                      MathUtil.clamp(
                          forwardSpeed + forwardsAssistEffort,
                          -drive.getMaxLinearSpeedMetersPerSec(),
                          drive.getMaxLinearSpeedMetersPerSec()),
                      MathUtil.clamp(
                          sidewaysSpeed + sidewaysAssistEffort, //
                          -drive.getMaxLinearSpeedMetersPerSec(),
                          drive.getMaxLinearSpeedMetersPerSec()),
                      MathUtil.clamp(
                          rotationSpeed + rotationAssistEffort,
                          -drive.getMaxAngularSpeedRadPerSec(),
                          drive.getMaxAngularSpeedRadPerSec())),
                  drive.getRotation()));
        },
        drive);
  }

  /*
   * translation:
   * +x is forward relative to the robot's new rotation
   * +y is left relative to the robot's new rotation
   */
  public static Pose2d rotateAndNudge(Pose2d pose, Translation2d translation, Rotation2d rotation) {
    Rotation2d rotation2d = pose.getRotation().rotateBy(rotation);
    Translation2d translation2d = pose.getTranslation().plus(translation.rotateBy(rotation2d));
    return new Pose2d(translation2d, rotation2d);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
              double rotationSpeed;
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  /*
    private static double calculateWantedSidewaysVelocity(
      Drive drive, double sidewaysError, double forwardSpeed) {
    double wantedSidewaysVelocityPID = sidewaysPID.calculate(sidewaysError);
    double forwardDisplacementToNote = getNearestReefSide(drive).getX(); // add Note_Forward_offset
    double maxTime;
    double minVelocity;
    if (forwardSpeed > 0 && forwardDisplacementToNote > 0) {
      maxTime = calculateTime(forwardSpeed, forwardDisplacementToNote);
      minVelocity = calculateVelocity(maxTime, getNearestReefSide(drive).getY());
      double wantedSidewaysVelocity =
          Math.max(Math.abs(wantedSidewaysVelocityPID), Math.abs(minVelocity))
              * (minVelocity / Math.abs(minVelocity));
      wantedSidewaysVelocity =
          MathUtil.clamp(
              wantedSidewaysVelocity,
              0.51 * -drive.getMaxLinearSpeedMetersPerSec(),
              0.51 * drive.getMaxLinearSpeedMetersPerSec());
      return wantedSidewaysVelocity;
    } else {
      return wantedSidewaysVelocityPID;
    }
  }
  private static double calculateTime(double velocity, double displacement) {
    double time = displacement / velocity;
    Logger.recordOutput("Time to note", time);
    return time;
  }
  private static double calculateVelocity(double time, double displacement) {
    double velocity = displacement / time;
    Logger.recordOutput("Velocity needed to note", velocity);
    return velocity;
  }
  */
}
