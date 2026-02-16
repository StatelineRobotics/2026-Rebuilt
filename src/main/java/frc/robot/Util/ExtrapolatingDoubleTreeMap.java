// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** Add your docs here. */
public class ExtrapolatingDoubleTreeMap {
    private final TreeMap<Double , Double > m_map;
    private final InverseInterpolator<Double > m_inverseInterpolator = InverseInterpolator.forDouble ();
    private final Interpolator<Double > m_interpolator = Interpolator.forDouble ();

    public ExtrapolatingDoubleTreeMap() {
        m_map = new TreeMap<>();
    }

    public Double  get(Double  key) {
        Double  val = m_map.get(key);
        if (val == null) {
            Double  ceilingKey = m_map.ceilingKey(key);
            Double  floorKey = m_map.floorKey(key);

            if (ceilingKey == null && floorKey == null) {
                return null;
            }
            if (ceilingKey == null) {
                ceilingKey = floorKey;
                floorKey = m_map.lowerKey(ceilingKey);
                double ceilingValue = m_map.get(ceilingKey);
                double floorValue = m_map.get(floorKey);
                double m = (ceilingValue - floorValue) / (ceilingKey - floorKey);
                double t = (key - floorKey) / (ceilingKey - floorKey);
                return t * m + floorValue;
            }
            if (floorKey == null) {
                floorKey = ceilingKey;
                ceilingKey = m_map.higherKey(floorKey);
                double ceilingValue = m_map.get(ceilingKey);
                double floorValue = m_map.get(floorKey);
                double m = (ceilingValue - floorValue) / (ceilingKey - floorKey);
                double t = (key - floorKey) / (ceilingKey - floorKey);
                return t * m + floorValue;
            }
            Double  floor = m_map.get(floorKey);
            Double  ceiling = m_map.get(ceilingKey);

            return m_interpolator.interpolate(
                floor, ceiling, m_inverseInterpolator.inverseInterpolate(floorKey, ceilingKey, key));
        } else {
            return val;
        }
    }

    /**
   * Inserts a key-value pair.
   *
   * @param key The key.
   * @param value The value.
   */
    public void put(Double  key, Double  value) {
        m_map.put(key, value);
    }

    /** Clears the contents. */
    public void clear() {
        m_map.clear();
    }
}
