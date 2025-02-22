// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoringCoral extends SequentialCommandGroup {
  /** Creates a new scoringCoral. */
  private final ScoralRollers scoralRollers;

  public scoringCoral(ScoralRollers m_scoralRollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.scoralRollers = m_scoralRollers;
    addCommands(scoralRollers.runVoltsCommmand(4));
    // new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.NO_CORAL));
  }
}
