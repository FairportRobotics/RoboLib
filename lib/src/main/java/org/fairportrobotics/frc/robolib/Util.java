package org.fairportrobotics.frc.robolib;

public abstract class Util {
    public static double rangePercent(double rangeMin, double rangeMax, double value) {
        double rangeWidth = rangeMax - rangeMin;
        double rangeOffset = value - rangeMin;
        return rangeOffset / rangeWidth;
    }
}
