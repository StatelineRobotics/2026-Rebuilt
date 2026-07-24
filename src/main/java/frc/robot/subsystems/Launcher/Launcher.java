// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Launcher.ShotCalculator.ShootingSolution;
import java.util.function.BooleanSupplier;

@Logged
public class Launcher extends SubsystemBase {

  public final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  public final Turret turret = new Turret();
  private CommandSwerveDrivetrain drivetrain;

  private ShootingSolution bestShootingSolution = new ShootingSolution(Degrees.of(0), Degrees.of(0), 0);

  public static Translation2d turretOffset = new Translation2d(Inches.of(-2.684942), Inches.of(-3.674131));

  public Trigger launcherReady = new Trigger(() -> flywheel.atTarget() && hood.atTarget() && turret.atTarget());

  private boolean currentlyShooting = false;
  private Pose2d mostRecentTarget = new Pose2d();
  private double[] tofDifferences = new double[20];
  private double distance = 0.0;
  private Pose2d recentPerp = new Pose2d();

  LinearFilter veloFilter = LinearFilter.movingAverage(2);
  double turretVelo = 0.0;

  // private LinearFilter distanceFilter = LinearFilter.movingAverage(20);

  /** Creates a new Launcher. */
  public Launcher(CommandSwerveDrivetrain Drivetrain, BooleanSupplier safeToSpin) {
    drivetrain = Drivetrain;
    turret.safeSpin = safeToSpin;

    // SmartDashboard.putData("Flywheel", flywheel);
    // SmartDashboard.putData("Hood", hood);
    // SmartDashboard.putData("Turret", turret);
    // SmartDashboard.putData("Launcher", this);

    setDefaultCommand(runToZero());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance = getTurretPose().getTranslation().getDistance(ShotCalculator.blueHubPose);

    var velocity = drivetrain.getFieldReletiveVelocity();
    bestShootingSolution = getBestShootingSolution(getTurretPose(), velocity);

    turretVelo = veloFilter.calculate(-velocity.omegaRadiansPerSecond);

    // var estDisplacement = new Translation2d(velocity.vxMetersPerSecond * 0.02, velocity.vyMetersPerSecond *
    // 0.02);
    // var estNextRobotPose = new Pose2d(
    //     drivetrain.getEstimatedPose().getTranslation().plus(estDisplacement),
    //     drivetrain
    //         .getEstimatedPose()
    //         .getRotation()
    //         .plus(new Rotation2d(velocity.omegaRadiansPerSecond * 0.02)));
    // var estNextBestShot = getBestShootingSolution(findTurretPose(estNextRobotPose), velocity);
    // AngularVelocity estTurretVelo =
    // estNextBestShot.turretAngle().minus(bestShootingSolution.turretAngle()).div(Seconds.of(0.02));
    // turret.setDesiredVelo(estTurretVelo);

    mostRecentTarget = ShotCalculator.mostRecentTarget;
  }

  private ShootingSolution getBestShootingSolution(Pose2d turretPose, ChassisSpeeds robotSpeeds) {
    recentPerp = new Pose2d(getTurretPose().getTranslation(), ShotCalculator.recentPerpVector.getAngle());
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      if (turretPose.getX() < 4.625594) {
        return currentlyShooting
            ? ShotCalculator.getSOTMhubSolution(turretPose, robotSpeeds)
            : ShotCalculator.getStaticHubSolution(turretPose);
      } else {
        return currentlyShooting
            ? ShotCalculator.getPassingSolution(turretPose, robotSpeeds)
            : ShotCalculator.getPassingSolution(turretPose, new ChassisSpeeds(0.0, 0.0, 0.0));
      }
    } else {
      if (turretPose.getX() > 16.540988 - 4.625594) {
        return currentlyShooting
            ? ShotCalculator.getSOTMhubSolution(turretPose, robotSpeeds)
            : ShotCalculator.getStaticHubSolution(turretPose);
      } else {
        return currentlyShooting
            ? ShotCalculator.getPassingSolution(turretPose, robotSpeeds)
            : ShotCalculator.getPassingSolution(turretPose, new ChassisSpeeds(0.0, 0.0, 0.0));
      }
    }
  }

  private Command expose(Command internal) {
    var proxied = internal.asProxy();
    proxied.addRequirements(this);
    return proxied;
  }

  public Command runToZero() {
    return expose((Commands.waitSeconds(1.0).andThen(flywheel.idleCommand()))
            .alongWith(hood.targetAngle(() -> Rotation.of(0)))
            .alongWith(turret.targetAngleWithVelocity(
                () -> bestShootingSolution.turretAngle(), () -> RadiansPerSecond.of(turretVelo))))
        .withName("Run to zero");
  }

  public Command everythingToZeroForReal() {
    return expose(flywheel.idleCommand()
        .alongWith(hood.targetAngle(() -> Rotation.of(0.0)))
        .alongWith(turret.targetAngle(() -> Rotation.of(0.0))));
  }

  public Command targetHub() {
    return expose(targetBest()).withName("TargetHub");
  }

  private Command targetBest() {
    return flywheel.runAtVelocity(() -> bestShootingSolution.flywheelSpeed())
        .alongWith(hood.targetAngle(() -> bestShootingSolution.hoodAngle()))
        .alongWith(turret.targetAngleWithVelocity(
            () -> bestShootingSolution.turretAngle(), () -> RadiansPerSecond.of(turretVelo)))
        .alongWith(Commands.startEnd(() -> currentlyShooting = true, () -> currentlyShooting = false));
  }

  public Command targetDashboard() {
    return expose(flywheel.runAtDashboardVelocity()
        .alongWith(hood.targetDashboardAngle()
            .alongWith(turret.targetAngle(() -> bestShootingSolution.turretAngle()))));
  }

  public Command testCommand() {
    return expose(flywheel.runAtVelocity(() -> 20.0)
        .alongWith(hood.targetAngle(() -> bestShootingSolution.hoodAngle()))
        .alongWith(turret.targetAngle(() -> bestShootingSolution.turretAngle()))
        .alongWith(Commands.startEnd(() -> currentlyShooting = true, () -> currentlyShooting = false)));
  }

  public Pose2d getTurretPose() {
    var curentPose = drivetrain.getEstimatedPose();
    return new Pose2d(
        curentPose.getTranslation().plus(turretOffset.rotateBy(curentPose.getRotation())),
        curentPose.getRotation());
  }

  public Pose2d findTurretPose(Pose2d robotPose) {
    return new Pose2d(
        robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation())),
        robotPose.getRotation());
  }

  public boolean turretReady() {
    return turret.atTarget();
  }
}
