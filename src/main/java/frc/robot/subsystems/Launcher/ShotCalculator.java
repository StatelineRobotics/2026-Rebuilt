// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Velocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Util.ExtrapolatingDoubleTreeMap;

/** Add your docs here. */
public class ShotCalculator {
    private static final Translation2d redHubPose = Translation2d.kZero;
    private static final Translation2d blueHubPose = Translation2d.kZero;
    private static Translation2d targetPose = Translation2d.kZero;
    private static final int NumItterations = 4;

    public record ShootingSolution(Rotation2d turretAngle, double hoodAngle, double flywheelSpeed) {}

    private static ExtrapolatingDoubleTreeMap tofMap = new ExtrapolatingDoubleTreeMap();

    static {
        tofMap.put(0.0, 0.0);
    }

    private static ExtrapolatingDoubleTreeMap hoodMap = new ExtrapolatingDoubleTreeMap();
    static {
        hoodMap.put(0.0, 0.0);
    }

    private static ExtrapolatingDoubleTreeMap flywheelMap = new ExtrapolatingDoubleTreeMap();
    static {
        flywheelMap.put(0.0, 0.0);
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

    public static ShootingSolution getStaticHubSolution(Pose2d robotPose) {
        if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
            targetPose = blueHubPose;
        } else {
            targetPose = redHubPose;
        }


        double distance = robotPose.getTranslation().getDistance(targetPose);
        Rotation2d turretAngle = targetPose.minus(robotPose.getTranslation()).getAngle();

        return new ShootingSolution(turretAngle, hoodMap.get(distance), flywheelMap.get(distance));
    }

    public static ShootingSolution getSOTMhubSolution(Pose2d robotPose, Translation2d robotVelocity) {
        if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
            targetPose = blueHubPose;
        } else {
            targetPose = redHubPose;
        }

        double distance = 0;
        //Itterate shot projection to hopefully converge on correct shot
        for (int i = 0; i < NumItterations; i++) {
            distance = robotPose.getTranslation().getDistance(targetPose);
            double tof = tofMap.get(distance);
            targetPose = targetPose.minus(robotVelocity.times(tof)); //Shift goal by predicted change in flight due to robot velocity
        }

        Rotation2d turretAngle = targetPose.minus(robotPose.getTranslation()).getAngle();

        //TODO: Check if shot is good (there are situations where it diverges or converges too slowly)

        return new ShootingSolution(turretAngle, hoodMap.get(distance), flywheelMap.get(distance));


    }

}
