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

public final class SubsystemConstants {

  public static final String CANIVORE_ID_STRING = "CAN Bus 2";
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final boolean tuningMode = true;

  public static final double NEAR_FAR_AT_REEF_OFFSET = -0.5;
  public static final double NEAR_FAR_AWAY_REEF_OFFSET = -0.9;
  public static final double LEFT_RIGHT_BRANCH_OFFSET = 0;

  public static class IntakeConstants {
    public static final double CURRENT_LIMIT = 40.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;
  }

  public static final class ShooterConstants {}

  public static class ElevatorConstants {
    public static final double CURRENT_LIMIT = 40.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double STOW_SETPOINT_INCH = 0.3;

    public static final double DEFAULT_THRESHOLD = 0.1;

    public static final double ELEVATOR_GEAR_RATIO = 12.;

    public static final double PROCESSOR_SETPOINT_INCHES = 0;

    // TODO:: NEED TO FIND
    public static final double L1_SETPOINT_INCHES = 0;
    public static final double L2_SETPOINT_INCHES = 7;
    public static final double L3_SETPOINT_INCHES = 14;
    public static final double L4_SETPOINT_INCHES = 27.3;
    public static final double BARGE_SETPOINT = 0;
  }

  public static final class ClimberConstants {
    public static final double CURRENT_LIMIT = 35.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double DEFAULT_THRESHOLD = 0.1;
    public static final double ARM_GEAR_RATIO = 1;

    public static final double STOW_SETPOINT_DEG = 90;
    public static final double DEPLOY_SETPOINT_DEG = 0;
  }

  public static class ScoralArmConstants {
    public static final double CURRENT_LIMIT = 35.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double DEFAULT_THRESHOLD = 1;
    public static final double ARM_GEAR_RATIO = (25.0 / 1) / (14.0 / 32);

    public static final double STOW_SETPOINT_DEG = 96;
    public static final double LOW_CORAL_SCORING_SETPOINT_DEG = 74;
    public static final double L4_CORAL_SCORING_SETPOINT_DEG = 63;
    public static final double BARGE_SETPOINT = 0;
  }

  public static class AlgaeScorerFlywheelConstants {
    public static final double FLYWHEEL_VELOCITY_DEGPERSEC = 10;
    public static final double THRESHOLD = 0;
    public static final boolean CURRENT_LIMIT_ENABLED = false;
    public static final double CURRENT_LIMIT = 0;
  }

  public static class ScoralRollersConstants {
    public static final String CoralScorerFlywheelConstants = null;
    public static final double FLYWHEEL_VELOCITY_DEGPERSEC = 10;
  }

  public static class LEDConstants {
    public static final int NUMBER_LEDS = 57 + 24 + 13;
  }

  public static enum LED_STATE {
    BLUE,
    RED,
    YELLOW,
    VIOLET,
    GREEN,
    GREY,
    PURPLE,
    PAPAYA_ORANGE,
    WILLIAMS_BLUE,
    HALF_FLASH_RED_HALF_FLASH_WHITE,
    FLASHING_WHITE,
    FLASHING_GREEN,
    FLASHING_RED,
    FLASHING_PURPLE,
    FLASHING_ORANGE,
    FLASHING_BLUE,
    FLASHING_GREY,
    FLASHING_PINK,
    FLASHING_YELLOW,
    FLASHING_BLACK,
    FLASHING_CYAN,
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
    SOURCE,
    PROCESSOR,
    L1,
    L2,
    L3,
    L4,
    CLIMB_STAGE_ONE,
    CLIMB_STAGE_TWO,
    HANG,
    // DEFAULT,
    SCORING_CORAL,
    INTAKE_ALGAE
    // READY_FOR_SCORE_REEF
  }

  public static final double CORAL_DIST = 4.0; // 1300; // CHANGE THIS
}
