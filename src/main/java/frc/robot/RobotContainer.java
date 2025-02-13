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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlignToReefAuto;
import frc.robot.commands.ApproachReefPerpendicular;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromSourceParallel;
import frc.robot.commands.IntakingAlgaeParallel;
import frc.robot.commands.SetClawLevel;
import frc.robot.commands.Stow;
import frc.robot.commands.algaeintoprocesser.AlgaeIntoProcesser;
import frc.robot.constants.FieldConstants;
// import frc.robot.commands.IntakeFromSource;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.coralIntake.flywheels.CoralIntakeSensorIO;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerArmIOSim;
import frc.robot.subsystems.coralscorer.CoralScorerArmIOTalonFX;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheelIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.led.LED_IOSim;
import frc.robot.subsystems.newalgaeintake.AlgaeIntakeArm;
import frc.robot.subsystems.newalgaeintake.AlgaeIntakeArmIOSim;
import frc.robot.subsystems.newalgaeintake.AlgaeIntakeArmIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.KeyboardInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final LED led;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController manipController = new CommandXboxController(1);
  private final Joystick joystikc = new Joystick(0);
  private final JoystickButton btn = new JoystickButton(joystikc, 4);
  private final KeyboardInputs keyboard = new KeyboardInputs(0);

  private final CoralScorerArm csArm;
  // private final CoralScorerFlywheel coralIntake;

  private final Elevator elevator;
  private final AlgaeIntakeArm algaeArm;
  private final Vision vision;

  private final CoralScorerFlywheel csFlywheel;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        csArm = new CoralScorerArm(new CoralScorerArmIOTalonFX(1));

        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));
        // TODO change lead, follower, gyro IDs, etc.
        elevator = new Elevator(new ElevatorIOTalonFX(0, 0));
        algaeArm = new AlgaeIntakeArm(new AlgaeIntakeArmIOTalonFX(0, 0, 0));
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralIntakeSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOCANdle(0, ""));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        csArm = new CoralScorerArm(new CoralScorerArmIOSim());

        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIOSim());
        algaeArm = new AlgaeIntakeArm(new AlgaeIntakeArmIOSim());
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralIntakeSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        csArm = new CoralScorerArm(new CoralScorerArmIOSim());
        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIO() {});
        algaeArm = new AlgaeIntakeArm(new AlgaeIntakeArmIOSim());
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralIntakeSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IO() {});
        break;
    }
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));

    NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));
    // autoChooser.addOption("toReefTest", AutoBuilder.buildAuto("toReefTest"));
    // Configure the button bindings
    // configureButtonBindings();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // keyboard.getCButton().whileTrue(new AlignToCage(drive));
    keyboard
        .getXButton()
        .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L2));
    keyboard
        .getZButton()
        .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L1));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.leftBumper().getAsBoolean(),
            () -> driveController.leftTrigger().getAsBoolean(),
            () -> driveController.rightTrigger().getAsBoolean(),
            () -> driveController.rightBumper().getAsBoolean(),
            () -> driveController.b().getAsBoolean(),
            () -> driveController.x().getAsBoolean()));

    // driveController.x().onTrue(new Stow(elevator, csArm));

    driveController
        .leftBumper()
        .onFalse(
            new ConditionalCommand(
                new ApproachReefPerpendicular(drive).withTimeout(2),
                new InstantCommand(),
                () -> (
                    !drive.isNearReef()
                    && drive.isAtReefSide())));

    driveController.a().onTrue(new SetClawLevel(ReefHeight.L1, elevator, csArm));
    driveController.a().onFalse(/*csFlywheel
                    .runVelocityCommand(2000)
                    .until(() -> csFlywheel.getLastCoralState() != CoralState.NO_CORAL).andThen*/ (new Stow(elevator, csArm)));

    driveController.y().onTrue(new AlgaeIntoProcesser(elevator, csArm, csFlywheel));
    driveController.y().onFalse(new Stow(elevator, csArm));

    // why is this like this?
    // driveController.leftBumper().onTrue(new InstantCommand(() -> drive.setNearestReefSide()));

    // driveController.leftBumper().whileTrue(new AutoAlignToSource(drive, led));
    // driveController.rightBumper().whileTrue(new AlignToReefAuto(drive, led));
    // driveController.rightTrigger().whileTrue(new AlignToProcessor(drive, led));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // controller.y().onTrue(csArm.setArmTarget(30, 0));

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // controller.y().whileTrue(elevator.setElevatorTarget(1.83, 1));
    // controller.y().whileFalse(elevator.setElevatorTarget(1, 1));

    // controller.x().whileTrue(csArm.setArmTarget(90, 1));
    // controller.x().whileFalse(csArm.setArmTarget(-90, 1));

    /*controller.b().whileTrue(algaeArm.setArmTarget(70, 2));
    controller.b().whileFalse(algaeArm.setArmTarget(20, 2));*/

    // manipController.rightTrigger().onTrue(new Stow(elevator, csArm));
    // driveController.a().whileTrue(new ReleaseClawParallel(scoringLevel, elevator, csArm,
    // csFlywheel));

    // manipController.a().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L1.height, 1)));
    // manipController.b().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L2.height, 1)));
    // manipController.x().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L3.height, 1)));
    // manipController.y().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 1)));

    manipController.a().onTrue(new SetClawLevel(FieldConstants.ReefHeight.L1, elevator, csArm));
    manipController.b().onTrue(new SetClawLevel(FieldConstants.ReefHeight.L2, elevator, csArm));
    manipController.x().onTrue(new SetClawLevel(FieldConstants.ReefHeight.L3, elevator, csArm));
    manipController.y().onTrue(new SetClawLevel(FieldConstants.ReefHeight.L4, elevator, csArm));
    manipController.a().onFalse(new Stow(elevator, csArm));
    manipController.b().onFalse(new Stow(elevator, csArm));
    manipController.x().onFalse(new Stow(elevator, csArm));
    manipController.y().onFalse(new Stow(elevator, csArm));

    // manipController.leftBumper().whileTrue(new AutoAlignToSource(drive, led));
    manipController.leftBumper().onTrue(new IntakeFromSourceParallel(csFlywheel, csArm, elevator));
    manipController.leftBumper().onFalse(new Stow(elevator, csArm));
    manipController
        .rightBumper()
        .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L1));
    manipController.rightBumper().onFalse(new Stow(elevator, csArm));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
