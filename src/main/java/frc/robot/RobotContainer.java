package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ApproachReefPerpendicular;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakingAlgae;
import frc.robot.commands.ReinitializingCommand;
import frc.robot.commands.Stow;
// import frc.robot.commands.IntakeFromSource;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.ClimberArmIOSim;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerArmIOSim;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheelIOSim;
import frc.robot.subsystems.coralscorer.CoralSensorIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.led.LED_IOSim;
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
  public static Drive drive;
  private LED led;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController manipController = new CommandXboxController(1);
  private final Joystick joystikc = new Joystick(0);
  private final JoystickButton btn = new JoystickButton(joystikc, 4);
  private final KeyboardInputs keyboard = new KeyboardInputs(0);

  private CoralScorerArm csArm;
  // private final CoralScorerFlywheel coralIntake;

  public static Elevator elevator;
  private ClimberArm climberArm;
  private Vision vision;
  SuperStructure superStructure;
  // public final Trigger elevatorBrakeTrigger;
  //   private final Trigger stateTrigger;
  private final Trigger slowModeTrigger;
  private CoralScorerFlywheel csFlywheel;

  // private final SequentialCommandGroup superStructureCommands;

  private SuperStructureState stateSelect() {
    return superStructure.getCurrentState();
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(),
        //         new ModuleIOTalonFX(TunerConstants.FrontLeft),
        //         new ModuleIOTalonFX(TunerConstants.FrontRight),
        //         new ModuleIOTalonFX(TunerConstants.BackLeft),
        //         new ModuleIOTalonFX(TunerConstants.BackRight));

        //  csArm = new CoralScorerArm(new CoralScorerArmIOTalonFX(1));

        // vision =
        //     new Vision(
        //         drive.getToPoseEstimatorConsumer(),
        //         new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
        //         new VisionIOPhotonVision("photon", new Transform3d()));
        // TODO change lead, follower, gyro IDs, etc.
        // elevator = new Elevator(new ElevatorIOTalonFX(8, 9));
        elevator = new Elevator(new ElevatorIOSim());

        // climberArm = new ClimberArm(new ClimberArmIOTalonFX(0, 0, 0));
        // csFlywheel =
        //     new CoralScorerFlywheel(
        //         new CoralScorerFlywheelIOSim(),
        //         new CoralSensorIO() {},
        //         CoralState.DEFAULT,
        //         AlgaeState.DEFAULT);
        // led = new LED(new LED_IOCANdle(0, ""));
        // superStructure = new SuperStructure(elevator, csArm, csFlywheel, drive, led);
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
        climberArm = new ClimberArm(new ClimberArmIOSim());
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOCANdle(0, "CAN Bus 2"));
        superStructure = new SuperStructure(drive, elevator, csArm, csFlywheel, led);
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
        climberArm = new ClimberArm(new ClimberArmIOSim());
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOSim());
        superStructure = new SuperStructure(drive, elevator, csArm, csFlywheel, led);
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
        climberArm = new ClimberArm(new ClimberArmIOSim());
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IO() {});
        superStructure = new SuperStructure(drive, elevator, csArm, csFlywheel, led);
        break;
    }

    // superStructureCommands = new
    // SelectCommand<>(Map.ofEntries(Map.entry(SuperStructureState.STOW,
    // superStructure.getSuperStructureCommand())), null);
    // Set up auto routines
    // NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));

    // NamedCommands.registerCommand(
    //     "L1",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "L2",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "L3",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "L4",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "STOW ", new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.STOW)));
    // NamedCommands.registerCommand(
    //     "SCORE",
    //     new SequentialCommandGroup(
    //         new InstantCommand(
    //             () -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "INTAKE",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SOURCE)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));

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
    autoChooser.addOption(
        "Top R3a", AutoBuilder.buildAuto("R3a(L3)-S1c-R2a(L3)-S2c-R1b(L3)-S3c-R6a(L3)"));
    autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));
    autoChooser.addDefaultOption("1.1 auto", AutoBuilder.buildAuto("1.1.auto"));
    /*
    NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));
    NamedCommands.registerCommand(
        "L1", new InstantCommand(() -> Super.setWantedState(SuperStructureState.L1)));
    NamedCommands.registerCommand(
        "L2", new InstantCommand(() -> Super.setWantedState(SuperStructureState.L2)));
    NamedCommands.registerCommand(
        "L3", new InstantCommand(() -> Super.setWantedState(SuperStructureState.L3)));
    NamedCommands.registerCommand(
        "L4", new InstantCommand(() -> Super.setWantedState(SuperStructureState.L4)));
    NamedCommands.registerCommand(
        "STOW ", new InstantCommand(() -> Super.setWantedState(SuperStructureState.STOW)));
    NamedCommands.registerCommand(
        "SCORE ",
        new InstantCommand(() -> Super.setWantedState(SuperStructureState.SCORING_CORAL)));
    NamedCommands.registerCommand(
        "INTAKE ", new InstantCommand(() -> Super.setWantedState(SuperStructureState.SOURCE)));
        */
    // autoChooser.addOption("toReefTest", AutoBuilder.buildAuto("toReefTest"));
    // Configure the button bindings
    // configureButtonBindings();
    // stateTrigger = new Trigger(() -> superStructure.changedStated());
    // elevatorBrakeTrigger = new Trigger(() -> RobotController.getUserButton());
    slowModeTrigger = new Trigger(() -> superStructure.elevatorExtended());
    // speedModeTrigger = new Trigger(() -> superStructure.elevatorExtended());
    configureButtonBindings();
    // test();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void test() {
    // driveController.b().onTrue(elevator.setElevatorTarget(20, 1));
    // driveController.b().onFalse(elevator.setElevatorTarget(0, 1));
    driveController.a().onTrue(led.setStateCommand(LED_STATE.BLUE));
    driveController.y().onTrue(led.setStateCommand(LED_STATE.GREEN));
    driveController.x().onTrue(led.setStateCommand(LED_STATE.FIRE));
  }

  private void configureButtonBindings() {
    slowModeTrigger.onTrue(new InstantCommand(() -> drive.enableSlowMode(true)));
    slowModeTrigger.onFalse(new InstantCommand(() -> drive.enableSlowMode(false)));

    driverControls();
    manipControls();
  }

  private void driverControls() {
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            superStructure,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.leftBumper().getAsBoolean(),
            () -> driveController.leftTrigger().getAsBoolean(),
            () -> driveController.rightTrigger().getAsBoolean()));

    driveController
        .leftBumper()
        .onFalse(
            new ConditionalCommand(
                new ApproachReefPerpendicular(drive, superStructure).withTimeout(2),
                new InstantCommand(),
                () ->
                    (!drive.isNearReef() && drive.isAtReefSide())
                        && superStructure.isTargetAReefState()));

    driveController
        .rightBumper()
        .onTrue(
            new WaitUntilCommand(() -> superStructure.atGoals())
                .andThen(
                    new ReinitializingCommand(
                        () -> superStructure.getSuperStructureCommand(),
                        elevator,
                        csArm,
                        csFlywheel,
                        drive,
                        led))
                .andThen(new InstantCommand(() -> superStructure.advanceWantedState())));

    driveController
        .a()
        .onTrue(
            new InstantCommand(
                () -> superStructure.setWantedState(SuperStructureState.CLIMB_STAGE_ONE)));
  }

  private void manipControls() {
    manipController
        .x()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));
    manipController
        .y()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)));
    manipController
        .a()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));
    manipController
        .b()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)));

    manipController
        .povDown()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
                .andThen(
                    new ReinitializingCommand(
                        () -> superStructure.getSuperStructureCommand(),
                        elevator,
                        csArm,
                        csFlywheel,
                        drive,
                        led)));
  }

  private void testControls() {
    // speedTrigger.onTrue(
    //     new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
    //         .andThen(superStructure.getSuperStructureCommand()));
    // keyboard.getCButton().whileTrue(new AlignToCage(drive));
    // keyboard
    //     .getXButton()
    //     .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L2));
    // keyboard
    //     .getZButton()
    //     .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L1));
    // stateTrigger.onTrue(superStructure.getSuperStructureCommand());

    // Default command, normal field-relative drive

    // driveController
    //     .a()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new ApproachReefPerpendicular(drive, superStructure).withTimeout(2),
    //             new InstantCommand(),
    //             () -> (!drive.isNearReef() && drive.isAtReefSide() &&
    // drive.isAtReefRotation())));

    // driveController.x().onTrue(new Stow(elevator, csArm));
    //   elevatorBrakeTrigger.onTrue(new InstantCommand(() -> elevator.breakMode(true), elevator));
    //  elevatorBrakeTrigger.onFalse(new InstantCommand(() -> elevator.breakMode(false)));

    //
    // driveController.y().onFalse(new InstantCommand(() -> csFlywheel.stop(), csFlywheel));
    // driveController
    //     .y()
    //     .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)));
    // driveController
    //     .povDown()
    //     .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));
    // driveController
    //     .povUp()
    //     .onTrue(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
    //             .andThen(superStructure.getSuperStructureCommand()));
    // driveController
    //     .a()
    //     .onFalse(
    //         new InstantCommand(
    //             () -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL)));
    // stateTrigger.onTrue(superStructure.getSuperStructureCommand());
    // driveController.a().onFalse(new SetClawLevel(ElevatorState.STOW, elevator, csArm));
    // driveController
    //     .rightBumper()
    //     .onTrue(new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.SOURCE)));

    // driveController
    //     .rightBumper()
    //     .onFalse(new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.STOW)));

    // // driveController
    // //   .a()
    // // .onTrue(new InstantCommand(() -> Super.setWantedState(SuperStructureState.L1)));

    // driveController.a().onTrue(new AlignToReefAuto(drive, led));
    // driveController
    //     .x()
    //     .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));
    // //  driveController
    // //    .a()
    // //   .onFalse(new InstantCommand(() -> Super.setWantedState(SuperStructureState.STOW)));

    // driveController.y().onTrue(new AlgaeIntoProcesser(elevator, csArm, csFlywheel));
    // driveController.y().onFalse(new Stow(elevator, csArm));

    // why is this like this?
    // driveController.leftBumper().onTrue(new InstantCommand(() -> drive.setNearestReefSide()));
    // driveController.rightBumper().onTrue(new SetClawLevel(ElevatorState.SOURCE, elevator,
    // csArm));
    // driveController.rightBumper().onFalse(new SetClawLevel(ElevatorState.STOW, elevator, csArm));
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

    // keyboard
    //    .getCButton()
    //    .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)));
    // keyboard
    //    .getVButton()
    //    .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));

    // manipController.leftBumper().whileTrue(new AutoAlignToSource(drive, led));
    // manipController.leftBumper().onTrue(new IntakeFromSourceParallel(csFlywheel, csArm,
    // elevator));
    // manipController.leftBumper().onFalse(new Stow(elevator, csArm));
    manipController.rightBumper().onTrue(new IntakingAlgae(elevator, csFlywheel, csArm));
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

  public CoralScorerArm getScoralArm() {
    return csArm;
  }
}
