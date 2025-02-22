// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.ScoralArmConstants;
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.ScoralRollersConstants;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.ElevatorConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFromSource extends Command {
  /** Creates a new IntakeFromSource. */
  private final ScoralRollers coralIntake;

  private final ScoralArm arm;
  private final Elevator elevator;
  private final LED led;

  public IntakeFromSource(ScoralRollers coralIntake, ScoralArm arm, Elevator elevator, LED led) {
    this.coralIntake = coralIntake;
    this.elevator = elevator;
    this.arm = arm;
    this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake, arm, elevator, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setElevatorTarget(2, ElevatorConstants.DEFAULT_THRESHOLD);
    arm.setArmGoal(ScoralArmConstants.INTAKE_SETPOINT_DEG);
    coralIntake.runVelocity(ScoralRollersConstants.FLYWHEEL_VELOCITY_DEGPERSEC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.setState(LED_STATE.FLASHING_PURPLE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // coralIntake.stop();
    arm.setArmGoal(ScoralArmConstants.STOW_SETPOINT_DEG);
    // elevator.setExtenderGoal(ElevatorConstants.RETRACT_SETPOINT_INCH);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralIntake.seesCoral() == CoralState.SENSOR
        || coralIntake.seesCoral() == CoralState.CURRENT;
  }
}
