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
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
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
  private static double sideWaysError = 0;
  private static double wantedSidewaysVelocity = 0;
  private static double wantedRotationVelocity = 0;
  private static double sidewaysAssistEffort = 0;
  private static double rotationAssistEffort = 0;
  private static double forwardConstantVelocity = 0;
  private static PIDController sidewaysPID =
      new PIDController(1.5, 0, 0, SubsystemConstants.LOOP_PERIOD_SECONDS);

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
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier reefAlignAssistSupplier) {
    return Commands.run(
        () -> {
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
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          double forwardSpeed = speeds.vxMetersPerSecond;

          double sidewaysSpeed = speeds.vyMetersPerSecond;

          double rotationSpeed = speeds.omegaRadiansPerSecond;

          double speedDebuf = 0.2;

          if (reefAlignAssistSupplier.getAsBoolean()) {
            wantedSidewaysVelocity =
                calculateWantedSidewaysVelocity(drive, sideWaysError, forwardSpeed);
            sidewaysAssistEffort = wantedSidewaysVelocity - sidewaysSpeed * speedDebuf;
          } else {
            forwardConstantVelocity = 0;
            wantedSidewaysVelocity = sidewaysSpeed;
            sidewaysAssistEffort = 0;
          }

          // drive.runVelocity(
          //     ChassisSpeeds.fromFieldRelativeSpeeds(
          //         speeds,
          //         isFlipped //WHY DO WE HAVE THIS "IS FLIPPED"
          //             ? drive.getRotation().plus(new Rotation2d(Math.PI))
          //             : drive.getRotation()));

          drive.runVelocity(
              new ChassisSpeeds(
                  MathUtil.clamp(
                      forwardSpeed + forwardConstantVelocity,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec()),
                  MathUtil.clamp(
                      sidewaysSpeed + sidewaysAssistEffort,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec()),
                  MathUtil.clamp(
                      rotationSpeed + rotationAssistEffort,
                      -drive.getMaxAngularSpeedRadPerSec(),
                      drive.getMaxAngularSpeedRadPerSec())));
        },
        drive);
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

  private static Pose2d getNearestReefSide(Drive drive) {
    Translation2d start = FieldConstants.Reef.center;
    Translation2d end = drive.getPose().getTranslation();
    Translation2d v = end.minus(start);
    Rotation2d angle = new Rotation2d(v.getX(), v.getY());

    // https://www.desmos.com/calculator/44dd9koglh

    // negate since branchPositions is CW not CCW
    // +6/12 since branchPositions starts at branch B not the +x axis
    double rawRotations = angle.getRotations();
    double adjustedRotations = -rawRotations + (7.0 / 12.0);

    // % 1 to just get the fractional part of the rotation
    // multiply by 12 before flooring so [0,1) maps to 0,1,2...10,11 evenly
    double fractionalRotation = adjustedRotations % 1;
    if (fractionalRotation < 0) {
      fractionalRotation++;
    }
    int index = (int) Math.floor(fractionalRotation * 12);

    Logger.recordOutput("align to reef target index", index);

    Pose2d result =
        FieldConstants.Reef.branchPositions.get(index).get(FieldConstants.ReefHeight.L1).toPose2d();

    // flip rotation
    Rotation2d rotation2d = result.getRotation().rotateBy(new Rotation2d(Math.PI));

    // back up target position (so it doesn't clip)
    // x is nearer/farther, y is sideways
    Translation2d offsetFromBranch = new Translation2d(-0.7, 0);
    offsetFromBranch = offsetFromBranch.rotateBy(rotation2d);
    Translation2d translation2d = result.getTranslation().plus(offsetFromBranch);

    result = new Pose2d(translation2d, rotation2d);

    Logger.recordOutput("align to reef target Pose2d", result);
    return result;
  }
}
