// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Util.ExtrapolatingDoubleTreeMap;

/** Add your docs here. */
public class ShotCalculator {
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
}
