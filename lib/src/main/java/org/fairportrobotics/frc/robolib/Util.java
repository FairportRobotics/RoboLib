package org.fairportrobotics.frc.robolib;

/**
 * Module defining general utility functions
 */
public abstract class Util {

    /**
     * <p> Get the mapping of value onto a range where rangeMin is 0, and
     * rangeMax is 1 (i.e. what percent of the way does value lie between 
     * rangeMin and rangeMax).</p>
     * 
     * <p> NOTE: rangeMin may be greater than rangeMax, but rangeMin must not
     * equal rangeMax.</p>
     *
     * @param rangeMin The 0 point of the range
     * @param rangeMax The 1 point of the range
     * @param value The value to check
     * @return The relative position of value in the range
     */
    public static double rangePercent(double rangeMin, double rangeMax, double value) {
        double rangeWidth = rangeMax - rangeMin;
        double rangeOffset = value - rangeMin;
        return rangeOffset / rangeWidth;
    }
}
