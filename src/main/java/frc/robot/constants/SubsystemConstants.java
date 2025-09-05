// Copyright 2021-2024 FRC 6328
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

// does not include swerve constants

package frc.robot.constants;

import frc.robot.util.LoggedTunableNumber;

public final class SubsystemConstants {
  public static boolean coralStuckMode = false;
  public static final String CANIVORE_ID_STRING = "CAN Bus 2";
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final boolean tuningMode = true;

  // public static final double NEAR_FAR_AT_REEF_OFFSET = -0.47;
  public static final double NEAR_FAR_AT_REEF_OFFSET = -0.482;
  public static final double NEAR_FAR_AWAY_REEF_OFFSET = -0.9;
  // more neg moves it to the right
  // public static final double LEFT_RIGHT_BRANCH_OFFSET = -0.015;
  public static final LoggedTunableNumber CORRECTION_LEFT_BRANCH_OFFSET =
      new LoggedTunableNumber("Auto Align/LEFT_BRANCH_OFFSET", 0.0);
  // new LoggedTunableNumber("Auto Align/LEFT_BRANCH_OFFSET", -0.023);
  public static final LoggedTunableNumber CORRECTION_RIGHT_BRANCH_OFFSET =
      new LoggedTunableNumber("Auto Align/RIGHT_BRANCH_OFFSET", 0.0);
  // new LoggedTunableNumber("Auto Align/RIGHT_BRANCH_OFFSET", -0.023);
  public static final double APPROACH_OFFSET_LEFT_RIGHT_OFFSET = -0.2;
  public static final double ADJUST_OFFSET_LEFT_RIGHT_OFFSET = -0.07;

  public static class IntakeConstants {
    public static final double CURRENT_LIMIT = 40.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;
  }

  public static final class ShooterConstants {}

  public static class ElevatorConstants {
    public static final double CURRENT_LIMIT = 40.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double STOW_SETPOINT_INCH = -0.02;

    public static final double DEFAULT_THRESHOLD = 0.1;

    public static final double ELEVATOR_GEAR_RATIO = 12.;

    public static final double PROCESSOR_SETPOINT_INCHES = 4;

    public static final double CANRAGE_ZERO_SETPOINT_INCHES = 2;

    // TODO:: NEED TO FIND
    public static final double L1_SETPOINT_INCHES = 3.8;
    public static final double L2_SETPOINT_INCHES = 7.5;
    // public static final double L2_SETPOINT_INCHES = 6.8;
    // public static final double L3_SETPOINT_INCHES = 14.5;
    public static final double L3_SETPOINT_INCHES = 14.2;
    public static final double L4_SETPOINT_INCHES = 27.4;
    // public static final double L4_SETPOINT_INCHES = 27;
    public static final double BARGE_SETPOINT = 27.5;
    public static final double GROUND_CORAL_ALGAE_SETPOINT_INCHES = 0.5;
  }

  public static final class ClimberConstants {
    public static final double CURRENT_LIMIT = 35.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double DEFAULT_THRESHOLD = 0.1;
    public static final double ARM_GEAR_RATIO = 1;

    public static final double STOW_SETPOINT_DEG = 108;
    public static final double DEPLOY_SETPOINT_DEG = -20;
  }

  public static final class WinchConstants {
    public static final double STATOR_CURRENT_LIMIT = 60.0;
  }

  public static class ScoralArmConstants {
    public static final double CURRENT_LIMIT = 50.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double DEFAULT_THRESHOLD = 1;
    public static final double ARM_GEAR_RATIO = (25.0 / 1) / (14.0 / 32);

    public static final double STOW_SETPOINT_DEG = 93.5;
    public static final double L1_CORAL_SCORING_SETPOINT_DEG = 98;
    public static final double L3_CORAL_SCORING_SETPOINT_DEG = 76.5;
    public static final double L2_CORAL_SCORING_SETPOINT_DEG = 76.5;
    public static final double L4_CORAL_SCORING_SETPOINT_DEG = 56;
    public static final double BARGE_BACK_SETPOINT_DEG = 69;
    public static final double BARGE_FORWARD_SETPOINT_DEG = 125;
    public static final double PROCESSOR_SETPOINT_DEG = 20;
    public static final double GROUND_CORAL_ALGAE_SETPOINT_DEG = 38;
  }

  public static class LEDConstants {
    public static final int NUMBER_LEDS = 57 + 24 + 13;
  }

  // TURRET TEMPLATE:
  public static final class TurretConstants {

    public static final double CURRENT_LIMIT = 50.0; // TODO: Update to actual value.
    public static final boolean CURRENT_LIMIT_ENABLED = true;
    public static double defaultPosDegs =
        0; // TODO: Update this to an actual value. This is the default position of the turret
    // (ideally where it would initialize as well).

    public static final double motorToothCount = 50;
    public static final double ringToothCount = 150;
    public static final double gearRatio = ringToothCount / motorToothCount;
  }

  public static enum LED_STATE {
    BLUE,
    RED,
    YELLOW,
    GREEN,
    GREY,
    PURPLE,
    PINK_LAVENDER,
    PAPAYA_ORANGE,
    FLASHING_GREEN,
    FLASHING_RED,
    FLASHING_BLUE,
    FIRE,
    OFF
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum CoralState {
    DEFAULT,
    NO_CORAL,
    SENSOR,
    CURRENT
  }

  public static enum AlgaeState {
    DEFAULT,
    NO_ALGAE,
    CURRENT,
  }

  public static enum ElevatorState {
    ZERO,
    STOW,
    L1,
    L2,
    L3,
    L4,
    SOURCE,
    PROCESSOR
  }

  public static enum ScoralFlywheelState {
    ZERO,
    INTAKING_CORAL,
    INTAKING_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE
  }

  public static enum ClimberState {
    STOW,
    STAGE_ONE,
    STAGE_TWO,
    HANG
  }

  public static enum AlignState {
    ALIGNING,
    IN_POSITION
  }

  public static enum DriveState {
    FULLSPEED,
    SLOW
  }

  public static enum SuperStructureState {
    // NONE,
    STOW,
    EJECT_ALGAE,
    SOURCE,
    PROCESSOR,
    BARGE_EXTEND,
    BARGE_SCORE,
    PROCESSOR_SCORE,
    L1,
    L2,
    L3,
    L4,
    SCORING_CORAL,
    L1_SCORING_CORAL,
    REEF_INTAKE_ALGAE,
    CORAL_INTAKE_ALGAE,
    STOW_ALGAE
  }

  public static final double CORAL_DIST = 4.0; // 1300; // CHANGE THIS
}
