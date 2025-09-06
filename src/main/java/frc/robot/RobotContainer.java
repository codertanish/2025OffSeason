package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AdjustToReefPost;
import frc.robot.commands.AlignToBarge;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.BargeExtend;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoToStowAfterProcessor;
import frc.robot.commands.GoToStowAuto;
import frc.robot.commands.GoToStowTeleOp;
import frc.robot.commands.IntakeAlgaeFromReef;
import frc.robot.commands.IntakingCoral;
import frc.robot.commands.MoveToProcessorSetpoints;
import frc.robot.commands.ReinitializingCommand;
import frc.robot.commands.Rumble;
import frc.robot.commands.ScoreAlgaeIntoBargeAuto;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SetClimberArmTarget;
import frc.robot.commands.SetElevatorTarget;
import frc.robot.commands.SetScoralArmTarget;
import frc.robot.commands.ToReefHeight;
import frc.robot.constants.RobotMap;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.ScoralArmConstants;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.ClimberArmIO;
import frc.robot.subsystems.climber.ClimberArmIOSim;
import frc.robot.subsystems.climber.ClimberArmIOTalonFX;
import frc.robot.subsystems.climber.Winch;
import frc.robot.subsystems.climber.WinchIO;
import frc.robot.subsystems.climber.WinchIOSim;
import frc.robot.subsystems.climber.WinchIOTalonFX;
import frc.robot.subsystems.commoniolayers.ArmIO;
import frc.robot.subsystems.commoniolayers.FlywheelIO;
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
import frc.robot.subsystems.scoral.DistanceSensorCANRangeIO;
import frc.robot.subsystems.scoral.DistanceSensorIO;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralArmIOSim;
import frc.robot.subsystems.scoral.ScoralArmIOTalonFX;
import frc.robot.subsystems.scoral.ScoralRollers;
import frc.robot.subsystems.scoral.ScoralRollersIOSim;
import frc.robot.subsystems.scoral.ScoralRollersIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSimArmBased;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private final ScoralArm scoralArm;
  private final ScoralRollers scoralRollers;
  public final Elevator elevator;
  public final ClimberArm climberArm;
  private final Winch winch;
  private final Vision vision;
  private final SuperStructure superStructure;
  private final Turret turret;
  private Command climbCommands;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController manipController = new CommandXboxController(1);
  // private final Joystick joystikc = new Joystick(0);
  // private final JoystickButton btn = new JoystickButton(joystikc, 4);
  // private final KeyboardInputs keyboard = new KeyboardInputs(0);
  private final KeyboardInputs keyboard = new KeyboardInputs(0);

  public final Trigger elevatorBrakeTrigger;
  private Trigger slowModeTrigger;
  private final Trigger reefAutoAlignTrigger;
  private final Trigger autoAlignDoneRumbleTrigger;
  private Trigger resetLimelight;
  private Trigger turnLimelightON;
  private Trigger autoAlignRelease;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autos;
  private DigitalInput brakeSwitch;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        turret =
            new Turret(
                new TurretIOTalonFX(
                    11,
                    1,
                    SubsystemConstants.TurretConstants.ringToothCount,
                    SubsystemConstants.TurretConstants.motorToothCount));
        brakeSwitch = new DigitalInput(RobotMap.BrakeSwitchIDs.brakeSwitchChannel);
        elevatorBrakeTrigger = new Trigger(() -> brakeSwitch.get());
        // // Real robot, instantiate hardware IO implementations
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    RobotMap.ElevatorIDs.leftElevatorID, RobotMap.ElevatorIDs.rightElevatorID));

        winch = new Winch(new WinchIOTalonFX(RobotMap.WinchIDs.leftWinchID));
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new DistanceSensorCANRangeIO(0, SubsystemConstants.CANIVORE_ID_STRING),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        scoralArm =
            new ScoralArm(
                new ScoralArmIOTalonFX(
                    RobotMap.CoralScorerArmIDs.coralScorerRotationID,
                    RobotMap.CoralScorerArmIDs.coralScorerRotationCANcoderID),
                new DistanceSensorCANRangeIO(0, "rio"));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-reef", () -> drive.getPose().getRotation()),
                new VisionIOLimelight("limelight-bakreef", () -> drive.getPose().getRotation())
                // new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                // new VisionIOPhotonVision("photon", new Transform3d())
                );

        climberArm =
            new ClimberArm(
                new ClimberArmIOTalonFX(
                    RobotMap.ClimbIDs.deployClimbID, RobotMap.ClimbIDs.deployClimbCANcoderID));
        scoralRollers =
            new ScoralRollers(
                new ScoralRollersIOTalonFX(RobotMap.CoralScorerArmIDs.coralScorerFlywheelID),
                new DistanceSensorCANRangeIO(
                    RobotMap.CoralScorerArmIDs.coralScorerCANrangeID, "rio"),
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOCANdle(RobotMap.ledIDs.CANdleID, "CAN Bus 2"));
        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm, winch);

        break;
      case SIM:
        turret = new Turret(new TurretIOSimArmBased());
        elevatorBrakeTrigger = new Trigger(() -> true);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new DistanceSensorIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        scoralArm = new ScoralArm(new ScoralArmIOSim(), new DistanceSensorIO() {});
        winch = new Winch(new WinchIOSim());

        Rotation3d bruh = new Rotation3d();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera 1 sim", new Transform3d(0, 0, 0, bruh), drive::getPose)
                // new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                // new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                // new VisionIOPhotonVision("photon", new Transform3d())
                );
        elevator = new Elevator(new ElevatorIOSim());
        scoralRollers =
            new ScoralRollers(
                new ScoralRollersIOSim(),
                new DistanceSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOSim());

        climberArm = new ClimberArm(new ClimberArmIOSim());

        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm, winch);
        break;

      default:
        // limelight = new PowerDistribution(23, ModuleType.kRev);
        turret = new Turret(new TurretIOSimArmBased());
        elevatorBrakeTrigger = new Trigger(() -> true);
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new DistanceSensorIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        scoralArm = new ScoralArm(new ArmIO() {}, new DistanceSensorIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera 1 sim", new Transform3d(0, 0, 0, new Rotation3d()), drive::getPose)
                // new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                // new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                // new VisionIOPhotonVision("photon", new Transform3d())
                );
        elevator = new Elevator(new ElevatorIO() {});
        scoralRollers =
            new ScoralRollers(
                new FlywheelIO() {},
                new DistanceSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IO() {});

        climberArm = new ClimberArm(new ClimberArmIO() {});

        winch = new Winch(new WinchIO() {});

        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm, winch);
        break;
    }
    // Set up auto routines
    // NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive,
    // led));

    NamedCommands.registerCommand(
        "L1",
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.SENSOR),
            new InstantCommand(() -> led.setState(LED_STATE.RED)),
            new ToReefHeight(
                elevator,
                scoralArm,
                SubsystemConstants.ElevatorConstants.L1_SETPOINT_INCHES,
                SubsystemConstants.ScoralArmConstants.L3_CORAL_SCORING_SETPOINT_DEG)));
    NamedCommands.registerCommand(
        "L2",
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.SENSOR),
            new InstantCommand(() -> led.setState(LED_STATE.RED)),
            new ToReefHeight(
                elevator,
                scoralArm,
                SubsystemConstants.ElevatorConstants.L2_SETPOINT_INCHES,
                SubsystemConstants.ScoralArmConstants.L2_CORAL_SCORING_SETPOINT_DEG)));
    NamedCommands.registerCommand(
        "L3",
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.SENSOR),
            new InstantCommand(() -> led.setState(LED_STATE.RED)),
            new ToReefHeight(
                elevator,
                scoralArm,
                SubsystemConstants.ElevatorConstants.L3_SETPOINT_INCHES,
                SubsystemConstants.ScoralArmConstants.L3_CORAL_SCORING_SETPOINT_DEG)));
    NamedCommands.registerCommand(
        "L4",
        new SequentialCommandGroup(
                new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.SENSOR),
                new InstantCommand(() -> led.setState(LED_STATE.RED)),
                new ToReefHeight(
                    elevator,
                    scoralArm,
                    SubsystemConstants.ElevatorConstants.L4_SETPOINT_INCHES,
                    SubsystemConstants.ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG))
            .withTimeout(4));

    NamedCommands.registerCommand("BARGE_EXTEND", new BargeExtend(elevator, scoralArm));

    NamedCommands.registerCommand(
        "BARGE_SCORE",
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> elevator.atGoal(2) && scoralArm.atGoal(2)),
            new ScoreAlgaeIntoBargeAuto(elevator, scoralArm, scoralRollers)));

    // NamedCommands.registerCommand(
    //     "SOURCE_INTAKE",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> led.setState(LED_STATE.GREY)),
    //         new ParallelRaceGroup(new WaitCommand(1.5), new IntakingCoral(scoralRollers)),
    //         // new WiggleWiggle(drive, scoralRollers),
    //         new InstantCommand(() -> led.setState(LED_STATE.BLUE))));

    NamedCommands.registerCommand(
        "SOURCE_INTAKE",
        new SequentialCommandGroup(
            new SetScoralArmTarget(
                scoralArm, SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG, 2),
            new InstantCommand(() -> led.setState(LED_STATE.GREY)),
            // new IntakingCoral(scoralRollers).withTimeout(0.25),
            new InstantCommand(() -> scoralRollers.runVolts(1.4)),
            new InstantCommand(() -> led.setState(LED_STATE.BLUE))));
    NamedCommands.registerCommand(
        "SEE_CORAL",
        new SequentialCommandGroup(
            // new WaitCommand(1.5),
            new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.SENSOR),
            new InstantCommand(() -> scoralRollers.stop())));

    NamedCommands.registerCommand(
        "STOW",
        new SequentialCommandGroup(
            new InstantCommand(() -> led.setState(LED_STATE.BLUE)),
            new GoToStowAuto(elevator, scoralArm, scoralRollers),
            new WaitCommand(0.1)));
    NamedCommands.registerCommand(
        "SCORE_CORAL",
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> elevator.atGoal(2) && scoralArm.atGoal(2)),
            new ScoreCoral(elevator, scoralArm, scoralRollers),
            new GoToStowTeleOp(elevator, scoralArm, scoralRollers),
            new WaitCommand(0.25)));
    NamedCommands.registerCommand(
        "SCORE_CORAL_NEW",
        new SequentialCommandGroup(
                new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.SENSOR),
                new ToReefHeight(
                    elevator,
                    scoralArm,
                    SubsystemConstants.ElevatorConstants.L4_SETPOINT_INCHES,
                    SubsystemConstants.ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG),
                new WaitUntilCommand(
                    () ->
                        elevator.atGoal(2)
                            && scoralArm.hasReachedGoal(
                                ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG)),
                scoralRollers.runVoltsCommmand(5),
                new WaitCommand(0.14))
            .withTimeout(3.5));

    NamedCommands.registerCommand(
        "PROCESSOR_SETPOINTS", new MoveToProcessorSetpoints(scoralArm, elevator));

    NamedCommands.registerCommand(
        "SCORE_PROCESSOR",
        new SequentialCommandGroup(scoralRollers.runVoltsCommmand(2), new WaitCommand(0.2)));

    NamedCommands.registerCommand(
        "STOW_AFTER_PROCESSOR", new GoToStowAfterProcessor(elevator, scoralArm, scoralRollers));

    NamedCommands.registerCommand(
        "INTAKE_ALGAE_FROM_REEF",
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> elevator.atGoal(2) && scoralArm.atGoal(2)),
            new ReinitializingCommand(
                () -> {
                  double height1 =
                      drive.getNearestParition(6) % 2 == 0
                          ? 6.5
                          : SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH;
                  double height2 = drive.getNearestParition(6) % 2 == 0 ? 9 : 2;
                  return new IntakeAlgaeFromReef(
                      drive, scoralArm, scoralRollers, elevator, led, height1, height2);
                })));

    // NamedCommands.registerCommand("Stow", new Stow(elevator, csArm));

    autos = new SendableChooser<>();

    autos.addOption("hello", AutoBuilder.buildAuto("hello"));
    autos.addOption("Processor", AutoBuilder.buildAuto("Processor"));
    // autos.addOption("BlueLeftPush", AutoBuilder.buildAuto("BlueLeftPush"));
    autos.addOption("CenterBarge", AutoBuilder.buildAuto("CenterBarge"));
    autos.addOption("CenterProcessor", AutoBuilder.buildAuto("CenterProcessor"));
    // autos.addOption("BlueMiddleLeft", AutoBuilder.buildAuto("BlueMiddleLeft"));
    // autos.addOption("BlueMiddleRight", AutoBuilder.buildAuto("BlueMiddleRight"));
    autos.addOption("Barge", AutoBuilder.buildAuto("Barge"));
    autos.addOption("BargeAlgae", AutoBuilder.buildAuto("BargeAlgae"));
    autos.addOption("ProcessorAlgae", AutoBuilder.buildAuto("ProcessorAlgae"));

    // autos.addOption("Wait6BlueLeftL2", AutoBuilder.buildAuto("Wait6BlueLeftL2"));
    // autos.addOption("Wait2BlueLeftL2", AutoBuilder.buildAuto("Wait2BlueLeftL2"));
    // autos.addOption("Wait6BlueRightL2", AutoBuilder.buildAuto("Wait6BlueRightL2"));
    // autos.addOption("Wait2BlueRightL2", AutoBuilder.buildAuto("Wait2BlueRightL2"));
    autos.addOption("BlueLeftL2", AutoBuilder.buildAuto("BlueLeftL2"));
    // autos.addOption("BlueLeftPushL2", AutoBuilder.buildAuto("BlueLeftPushL2"));
    // autos.addOption("BlueRightL2", AutoBuilder.buildAuto("BlueRightL2"));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autos);

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
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

    slowModeTrigger = new Trigger(() -> superStructure.shouldSlowMode());

    reefAutoAlignTrigger =
        new Trigger(
            () ->
                drive.shouldRunReefCommand()
                    && (driveController.rightTrigger().getAsBoolean()
                        || driveController.leftTrigger().getAsBoolean()));
    autoAlignDoneRumbleTrigger =
        new Trigger(
            () ->
                (drive.isReefAutoAlignDone
                        && superStructure.atGoals()
                        && superStructure.isCurrentAReefState())
                    || (drive.isBargeAutoAlignDone));
    resetLimelight = new Trigger(() -> SmartDashboard.getBoolean("Reset", false));
    turnLimelightON = new Trigger(() -> SmartDashboard.getBoolean("Enable", false));

    autoAlignRelease =
        new Trigger(
            () ->
                driveController.leftTrigger().getAsBoolean()
                    || driveController.rightTrigger().getAsBoolean());

    // configureButtonBindings();
    // test();
    turretTest();
  }

  private void turretTest() {
    keyboard.getXButton().onTrue(new InstantCommand(() -> turret.setPositionDegs(90, 10)));
    keyboard.getCButton().onTrue(new InstantCommand(() -> scoralArm.setPositionDegs(20, 20)));
  }

  private void test() {
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         superStructure,
    //         led,
    //         () -> -driveController.getLeftY(),
    //         () -> -driveController.getLeftX(),
    //         () -> -driveController.getRightX(),
    //         () -> driveController.leftBumper().getAsBoolean()));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            superStructure,
            led,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    driveController
        .leftTrigger()
        .onTrue(
            new AdjustToReefPost(
                drive,
                scoralArm,
                superStructure,
                false,
                () -> driveController.leftTrigger().getAsBoolean()));
    // driveController.b().onTrue(new SetScoralArmTarget(scoralArm, 20, 2));
  }

  private void configureButtonBindings() {
    resetLimelight.onTrue(vision.resetLimelight().ignoringDisable(true));
    turnLimelightON.onTrue(vision.activateLimelight().ignoringDisable(true));
    slowModeTrigger.onTrue(new InstantCommand(() -> drive.enableSlowMode(true)));
    slowModeTrigger.onFalse(new InstantCommand(() -> drive.enableSlowMode(false)));
    reefAutoAlignTrigger.onTrue(
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.SENSOR),
            new ReinitializingCommand(
                () -> {
                  superStructure.setWantedState(superStructure.getLastReefState());

                  return superStructure.getSuperStructureCommand();
                })));

    autoAlignDoneRumbleTrigger.onTrue(
        new InstantCommand(() -> led.setState(LED_STATE.PAPAYA_ORANGE))
            .andThen(new Rumble(driveController))
            .andThen(new InstantCommand(() -> drive.isBargeAutoAlignDone = false))
            .andThen(new InstantCommand(() -> drive.isReefAutoAlignDone = false)));

    elevatorBrakeTrigger.onTrue(
        new InstantCommand(() -> elevator.setBrake(false)).ignoringDisable(true));
    elevatorBrakeTrigger.onFalse(
        new InstantCommand(() -> elevator.setBrake(true)).ignoringDisable(true));

    driverControls();
    manipControls();
  }

  private void driverControls() {
    driveController
        .start()
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
            led,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    driveController
        .leftTrigger()
        .and(() -> !driveController.rightTrigger().getAsBoolean())
        .whileTrue(
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new IntakingCoral(scoralRollers),
                    new InstantCommand(),
                    () -> scoralRollers.seesCoral() != CoralState.SENSOR),
                new SequentialCommandGroup(
                    new ApproachReef(
                        drive,
                        led,
                        superStructure,
                        false,
                        () -> driveController.leftTrigger().getAsBoolean()),
                    new WaitUntilCommand(
                        () -> superStructure.atGoals() && superStructure.isCurrentAReefState()),
                    new AdjustToReefPost(
                        drive,
                        scoralArm,
                        superStructure,
                        false,
                        () -> driveController.leftTrigger().getAsBoolean()))));
    driveController
        .rightTrigger()
        .and(() -> !driveController.leftTrigger().getAsBoolean())
        .whileTrue(
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new IntakingCoral(scoralRollers),
                    new InstantCommand(),
                    () -> scoralRollers.seesCoral() != CoralState.SENSOR),
                new SequentialCommandGroup(
                    new ApproachReef(
                        drive,
                        led,
                        superStructure,
                        true,
                        () -> driveController.rightTrigger().getAsBoolean()),
                    new WaitUntilCommand(
                        () -> superStructure.atGoals() && superStructure.isCurrentAReefState()),
                    new AdjustToReefPost(
                        drive,
                        scoralArm,
                        superStructure,
                        true,
                        () -> driveController.rightTrigger().getAsBoolean()))));

    autoAlignRelease.onTrue(new InstantCommand(() -> led.setState(LED_STATE.RED)));

    driveController
        .rightBumper()
        .onTrue(
            new WaitUntilCommand(() -> superStructure.atGoals())
                .andThen(
                    new ReinitializingCommand(
                        () -> superStructure.getSuperStructureCommand(),
                        elevator,
                        climberArm,
                        scoralArm,
                        scoralRollers,
                        led))
                .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
                .andThen(new InstantCommand(() -> superStructure.nextState())));

    // driveController.x().onTrue(climbCommands);
    // driveController.x().onFalse(new InstantCommand(() -> winch.stop()));
    driveController
        .a()
        .onTrue(
            new ParallelCommandGroup(
                new SetScoralArmTarget(scoralArm, 29, 2),
                new SetClimberArmTarget(climberArm, 60, 2)));

    driveController.x().onTrue(new InstantCommand(() -> winch.runVolts(-5)));
    driveController.x().onFalse(new InstantCommand(() -> winch.stop()));

    driveController
        .b()
        .whileTrue(
            new InstantCommand(
                    () -> superStructure.setWantedState(SuperStructureState.REEF_INTAKE_ALGAE))
                .andThen(
                    new ReinitializingCommand(
                            () -> superStructure.getSuperStructureCommand(),
                            elevator,
                            scoralArm,
                            scoralRollers,
                            led)
                        .andThen(new InstantCommand(() -> superStructure.nextState()))));

    driveController
        .leftBumper()
        .whileTrue(
            new InstantCommand(
                    () -> superStructure.setWantedState(SuperStructureState.BARGE_EXTEND))
                .andThen(
                    new AlignToBarge(
                        drive,
                        led,
                        superStructure,
                        () -> driveController.leftBumper().getAsBoolean())));

    driveController
        .leftBumper()
        .onFalse(new InstantCommand(() -> led.setState(LED_STATE.PINK_LAVENDER)));

    driveController
        .povDown()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
                .andThen(new InstantCommand(() -> scoralArm.setConstraints(150, 300)))
                .andThen(
                    new ReinitializingCommand(
                            () -> superStructure.getSuperStructureCommand(),
                            elevator,
                            scoralArm,
                            scoralRollers,
                            led)
                        .andThen(new InstantCommand(() -> superStructure.nextState()))));

    driveController
        .back()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> superStructure.setCurrentState(SuperStructureState.L4)),
                new SetElevatorTarget(
                    elevator, SubsystemConstants.ElevatorConstants.L4_SETPOINT_INCHES, 2),
                new SetScoralArmTarget(
                    scoralArm,
                    SubsystemConstants.ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG,
                    2),
                new InstantCommand(
                    () -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL))));
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
                .andThen(new InstantCommand(() -> scoralArm.setConstraints(150, 300)))
                .andThen(
                    new ReinitializingCommand(
                        () -> superStructure.getSuperStructureCommand(),
                        elevator,
                        scoralArm,
                        scoralRollers,
                        led))
                .andThen(new InstantCommand(() -> superStructure.nextState())));

    manipController
        .rightBumper()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SOURCE)));

    manipController
        .povLeft()
        .onTrue(new InstantCommand(() -> superStructure.enableAlgaeMode(true)));

    manipController
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> superStructure.setWantedState(SuperStructureState.BARGE_EXTEND)));

    manipController
        .povRight()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.PROCESSOR)));

    manipController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> superStructure.setWantedState(SuperStructureState.CORAL_INTAKE_ALGAE)));

    manipController
        .leftTrigger()
        .onTrue(
            new ParallelCommandGroup(
                new SetScoralArmTarget(scoralArm, 29, 2),
                new SetClimberArmTarget(climberArm, 60, 2)));

    // not sure if it works
    // manipController
    //     .start()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new SequentialCommandGroup(
    //                 scoralArm.setArmTarget(
    //                     SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG, 2),
    //                 new WaitUntilCommand(() -> scoralArm.atGoal(2)),
    //                 new ZeroElevatorCANRange(elevator)),
    //             new InstantCommand(),
    //             () -> climberArm.isAt(SubsystemConstants.ClimberConstants.STOW_SETPOINT_DEG,
    // 3)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public ScoralArm getScoralArm() {
    return scoralArm;
  }

  public Drive getDrive() {
    return drive;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public SuperStructure getSuperStructure() {
    return superStructure;
  }

  public LED getLED() {
    return led;
  }

  public ScoralRollers getScoralRollers() {
    return scoralRollers;
  }

  public ClimberArm getClimber() {
    return climberArm;
  }
}
