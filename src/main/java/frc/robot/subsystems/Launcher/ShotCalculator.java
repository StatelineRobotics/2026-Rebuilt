// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Util.ExtrapolatingDoubleTreeMap;

/** Add your docs here. */
@Logged
public class ShotCalculator {
  private static final Translation2d redHubPose = new Translation2d(11.915394, 4.021328);
  public static final Translation2d blueHubPose = new Translation2d(4.625594, 4.021328);
  public static final Translation2d blueRightPass = new Translation2d(0.5, 2.0);
  private static final Translation2d blueLeftPass = new Translation2d(0.5, 6);
  private static final Translation2d redRightPass = new Translation2d(15, 2.0);
  private static final Translation2d redLeftPass = new Translation2d(15, 6.0);
  public static Pose2d mostRecentTarget = new Pose2d();

  // Normalized vector perpendicular to vector from center of robot to turret
  private static Translation2d turretPerpVector = new Translation2d(
      -Launcher.turretOffset.getY() / Launcher.turretOffset.getNorm(),
      Launcher.turretOffset.getX() / Launcher.turretOffset.getNorm());
  public static Translation2d recentPerpVector = turretPerpVector;

  private static Translation2d targetPose = Translation2d.kZero;
  private static final int NumItterations = 20;
  public static final double[] tofDifference = new double[NumItterations];

  private static final NetworkTableEntry tofMult =
      NetworkTableInstance.getDefault().getEntry("/adjustments/tofMult");
  private static final NetworkTableEntry tofAdd =
      NetworkTableInstance.getDefault().getEntry("/adjustments/tofAdd");

  static {
    tofMult.getTopic().genericPublish("double");
    tofMult.getTopic().setPersistent(true);
    tofAdd.getTopic().genericPublish("double");
    tofAdd.getTopic().setPersistent(true);
  }

  public record ShootingSolution(Angle turretAngle, Angle hoodAngle, double flywheelSpeed) {}

  private static ExtrapolatingDoubleTreeMap tofMap = new ExtrapolatingDoubleTreeMap();

  static {
    tofMap.put(2.17, 1.26);
    tofMap.put(2.51, 1.31);
    tofMap.put(3.049, 1.32);
    tofMap.put(3.515, 1.33);
    tofMap.put(4.011, 1.35);
    tofMap.put(4.5, 1.36);
    tofMap.put(5.0, 1.37);
    tofMap.put(5.4, 1.38);
  }

  private static InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  static {
    hoodMap.put(2.17, 25.0);
    hoodMap.put(2.51, 27.0);
    hoodMap.put(3.049, 29.0);
    hoodMap.put(3.515, 30.0);
    hoodMap.put(4.011, 31.0);
    hoodMap.put(4.5, 33.4);
    hoodMap.put(5.0, 35.0);
    hoodMap.put(5.4, 37.0);
    hoodMap.put(30.0, 37.0);
  }

  private static InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

  static {
    flywheelMap.put(2.17, 73.0);
    flywheelMap.put(2.51, 73.0);
    flywheelMap.put(3.049, 74.0);
    flywheelMap.put(3.515, 77.0);
    flywheelMap.put(4.011, 80.0);
    flywheelMap.put(4.5, 82.0);
    flywheelMap.put(5.4, 87.0);
    flywheelMap.put(5.0, 85.0);
    flywheelMap.put(8.0, 93.0);
  }

  private static InterpolatingDoubleTreeMap passHoodMap = new InterpolatingDoubleTreeMap();

  static {
    passHoodMap.put(5.5, 40.0);
    passHoodMap.put(7.0, 40.0);
    passHoodMap.put(7.5, 42.0);
    passHoodMap.put(8.5, 48.0);
    passHoodMap.put(20.0, 48.0);
  }

  private static InterpolatingDoubleTreeMap passFlywheelMap = new InterpolatingDoubleTreeMap();

  static {
    passFlywheelMap.put(5.5, 75.0);
    passFlywheelMap.put(6.0, 80.0);
    passFlywheelMap.put(7.0, 88.0);
    passFlywheelMap.put(7.5, 88.0);
    passFlywheelMap.put(8.5, 92.0);
    passFlywheelMap.put(9.5, 100.0);
    passFlywheelMap.put(10.5, 115.0);
    passFlywheelMap.put(20.0, 115.0);
  }

  public static double getFlywheelSpeed(double distance) {
    return flywheelMap.get(distance);
  }

  public static double getHoodPosition(double distance) {
    return hoodMap.get(distance);
  }

  public static double getTOF(double distance) {
    return tofMap.get(distance);
  }

  public static ShootingSolution getStaticHubSolution(Pose2d turretPose) {
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      targetPose = blueHubPose;
    } else {
      targetPose = redHubPose;
    }

    Translation2d difference = targetPose.minus(turretPose.getTranslation());
    double distance = difference.getNorm();

    Angle turretAngle =
        difference.getAngle().minus(turretPose.getRotation()).getMeasure();

    return new ShootingSolution(turretAngle, Degrees.of(hoodMap.get(distance)), flywheelMap.get(distance));
  }

  public static ShootingSolution getPassingSolution(Pose2d turretPose, ChassisSpeeds robotVelocity) {
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      targetPose = (turretPose.getY() < 4.0) ? blueRightPass : blueLeftPass;
    } else {
      targetPose = (turretPose.getY() < 4.0) ? redRightPass : redLeftPass;
    }

    Translation2d currentTurretVector = turretPerpVector.rotateBy(turretPose.getRotation());
    recentPerpVector = currentTurretVector;
    Translation2d rotLinearVelocity =
        currentTurretVector.times(robotVelocity.omegaRadiansPerSecond * Launcher.turretOffset.getNorm());
    double timeOfFlight = 1.125;

    double offsetX = (robotVelocity.vxMetersPerSecond + rotLinearVelocity.getX()) * linearDragComp(timeOfFlight);
    double offsetY = (robotVelocity.vyMetersPerSecond + rotLinearVelocity.getY()) * linearDragComp(timeOfFlight);
    targetPose = targetPose.minus(new Translation2d(offsetX, offsetY));

    Translation2d difference = targetPose.minus(turretPose.getTranslation());
    double distance = difference.getNorm();

    Angle turretAngle =
        difference.getAngle().minus(turretPose.getRotation()).getMeasure();

    return new ShootingSolution(turretAngle, Degrees.of(passHoodMap.get(distance)), passFlywheelMap.get(distance));
  }

  public static ShootingSolution getSOTMhubSolution(Pose2d turretPose, ChassisSpeeds robotVelocity) {
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      targetPose = blueHubPose;
    } else {
      targetPose = redHubPose;
    }

    Translation2d currentTurretVector = turretPerpVector.rotateBy(turretPose.getRotation());
    recentPerpVector = currentTurretVector;
    Translation2d rotLinearVelocity =
        currentTurretVector.times(robotVelocity.omegaRadiansPerSecond * Launcher.turretOffset.getNorm());

    Translation2d launcherPosition = turretPose.getTranslation();
    double lookaheadLauncherToTargetDistance = targetPose.getDistance(launcherPosition);

    double lastTOF = 0.0;
    double timeOfFlight = tofMap.get(lookaheadLauncherToTargetDistance);
    double tofDifference = Math.abs(timeOfFlight - lastTOF);
    Translation2d lookaheadPose = targetPose;

    // Itterate shot projection to hopefully converge on correct shot
    for (int i = 0; i < NumItterations && tofDifference > 1.0 / 1000.0; i++) {
      timeOfFlight = tofMap.get(lookaheadLauncherToTargetDistance);
      timeOfFlight = timeOfFlight * tofMult.getDouble(1.0) + tofAdd.getDouble(0.0);
      double offsetX =
          (robotVelocity.vxMetersPerSecond + rotLinearVelocity.getX()) * linearDragComp(timeOfFlight);
      double offsetY =
          (robotVelocity.vyMetersPerSecond + rotLinearVelocity.getY()) * linearDragComp(timeOfFlight);
      lookaheadPose = targetPose.minus(new Translation2d(offsetX, offsetY));
      lookaheadLauncherToTargetDistance = launcherPosition.getDistance(lookaheadPose);
      lastTOF = timeOfFlight;
      tofDifference = Math.abs(timeOfFlight - lastTOF);
    }

    targetPose = lookaheadPose;
    var difference = targetPose.minus(turretPose.getTranslation());
    Angle turretAngle =
        difference.getAngle().minus(turretPose.getRotation()).getMeasure();
    var distance = lookaheadLauncherToTargetDistance;

    // TODO: Check if shot is good (there are situations where it diverges or converges too slowly)

    return new ShootingSolution(turretAngle, Degrees.of(hoodMap.get(distance)), flywheelMap.get(distance));
  }

  private static double linearDragComp(double t) {
    return t;
    // double k = 0.02;
    // return (1.0 - Math.exp(-k * t)) / k;
  }
}
