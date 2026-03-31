// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher.Launcher;
import frc.robot.subsystems.Vision;

@Logged
public class RobotContainer {
  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double shootingMaxSpeed = MaxSpeed * 0.1;
  private double currentMax = MaxSpeed;
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private double shootingMaxRotation = MaxAngularRate * 0.2;
  private double currentMaxRotation = MaxAngularRate;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.001)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final Pose2d pose = new Pose2d(10.0, 2.0, Rotation2d.kZero);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Launcher launcher = new Launcher(drivetrain);
  public final Intake intake = new Intake();
  public final Indexer indexer = new Indexer();
  public final Vision vision =
      new Vision(drivetrain::addVisionMeasurement, () -> pose, drivetrain::getPigeonRotation);

  public final AutoTagger tagger = new AutoTagger(drivetrain, getShootCommand(), intake.autoAgitate());

  private final SendableChooser<Command> autoChooser;

  Pose2d blinePose = new Pose2d();

  public RobotContainer() {
    doNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putNumber("Auto Delay", 0.0);
    SmartDashboard.putData("Auto Tag", tagger.getChosser());
    SmartDashboard.putData("testLauncher", launcher.testCommand().alongWith(indexer.runIndexer()));

    FollowPath.setPoseLoggingConsumer((pair) -> blinePose = pair.getSecond());

    // SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureBindings() {
    RobotModeTriggers.disabled().negate().onTrue(launcher.idle());
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> drive.withVelocityX((Math.pow(-joystick.getLeftY(), 3))
                    * currentMax) // Drive forward with negative Y (forward)
                .withVelocityY(Math.pow(-joystick.getLeftX(), 3) * currentMax
                    + drivetrain.getTrenchOffset()) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX()
                    * currentMaxRotation) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
    // ));

    joystick.rightTrigger()
        .whileTrue(getShootCommand().alongWith(Commands.runOnce(() -> {
          currentMax = shootingMaxSpeed;
          currentMaxRotation = shootingMaxRotation;
        })))
        .whileFalse(indexer.idleCommand().alongWith(Commands.runOnce(() -> {
          currentMax = MaxSpeed;
          currentMaxRotation = MaxAngularRate;
        })));
    joystick.leftTrigger().whileTrue(intake.intakeCommand()).onFalse(intake.idleDeployed());
    joystick.leftBumper().whileTrue(intake.reverseIntake()).onFalse(intake.idleDeployed());
    joystick.rightBumper().whileTrue(indexer.reverseIndexer()).whileFalse(indexer.idleCommand());

    joystick.a().whileTrue(intake.agitate()).onFalse(intake.deployCommand());

    // Reset the field-centric heading on left bumper press.
    // joystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.a().whileTrue(launcher.flywheel.sysIdDynamic(Direction.kForward));
    // joystick.b().whileTrue(launcher.flywheel.sysIdDynamic(Direction.kReverse));
    // joystick.x().whileTrue(launcher.flywheel.sysIdQuasistatic(Direction.kForward));
    // joystick.y().whileTrue(launcher.flywheel.sysIdQuasistatic(Direction.kReverse));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void doNamedCommands() {
    Path returnPath = new Path("Left Bump Return");

    NamedCommands.registerCommand(
        "runShooter",
        Commands.parallel(
                launcher.targetHub(),
                Commands.waitUntil(launcher.launcherReady).andThen(indexer.antiJamRun()))
            .asProxy());
    NamedCommands.registerCommand("runIntake", intake.intakeCommand().asProxy());
    NamedCommands.registerCommand(
        "LeftBump", drivetrain.pathBuilder.build(returnPath).until(drivetrain::inAllianceZone));
    NamedCommands.registerCommand("leftReturn", tagger.getLeftZone());
    new EventTrigger("runIntake").onTrue(intake.intakeCommand().asProxy());
    new EventTrigger("shoot").whileTrue(getShootCommand().asProxy());
  }

  public Command getAutonomousCommand() {
    return // getCombinedCommand();
    Commands.sequence(autoChooser.getSelected(), tagger.getChosser().getSelected());
  }

  public Command getCombinedCommand() {
    return autoChooser.getSelected();
  }

  public Command getShootCommand() {
    return launcher.targetHub()
        .alongWith(Commands.waitUntil(launcher.launcherReady).andThen(indexer.antiJamRun()));
  }
}
