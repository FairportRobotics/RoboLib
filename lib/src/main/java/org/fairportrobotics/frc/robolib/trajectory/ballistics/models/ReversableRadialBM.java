package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * Base class for Ballistic models where translation is radial only (i.e. no side-slip) and
 * each radial position corresponds to exactly one time (e.g. a simple parabolic trajectory)
 */
public abstract class ReversableRadialBM<P> extends BallisticModel<P> {

    public ReversableRadialBM(Translation3d posLaunch, Translation3d posTarget) {
        super(posLaunch, posTarget);
    }

    //
    //  Reverse functions
    //

    /**
     * Get the time taken to travel a given radial distance from posLaunch towards posTarget
     *
     * @param candidate The generated velocity to compute against
     * @param radialDistance The radial distance from posLaunch to posTarget to solve for
     * @return The time it takes to travel radialDistance from posTarget towards posTarget
     */
    public abstract Time getTimeAtRadius(Velocity3d candidate, Distance radialDistance);


    /**
     * Core function converting linear position (relative to posLaunch) to time
     * using this.getTimeAtRadius
     *
     * @param candidate The candidate velocity to compute for
     * @param distance Horizontal position (relative to posLaunch) to test for
     * @param isXAxis If true, evaluate for the X axis
     * @return The time to the given position
     */
    private Time getTimeAtHoriAxis(Velocity3d candidate, Distance distance, boolean isXAxis) {
        double azimuthRadians = candidate.getAzimuthAngle().in(Units.Radians);
        Distance radialDistance = Units.Meters.of(
            distance.in(Units.Meters) / ((isXAxis) ? Math.cos(azimuthRadians) : Math.sin(azimuthRadians))
        );
        return this.getTimeAtRadius(candidate, radialDistance);
    }


    /**
     * Get the time taken to reach a given X position (relative to posLaunch)
     * for a given launch velocity
     * 
     * @param candidate The launch velocity
     * @param xDistance The X position (relative to posLaunch)
     * @return The time to the given position
     */
    public Time getTimeAtXRelative(Velocity3d candidate, Distance xDistance) {
        return getTimeAtHoriAxis(candidate, xDistance, true);
    }

    /**
     * Get the time taken to reach a given Y position (relative to posLaunch)
     * for a given launch velocity
     *
     * @param candidate The launch velocity
     * @param yDistance The Y position (relative to posLaunch)
     * @return The time to the given position
     */
    public Time getTimeAtYRelative(Velocity3d candidate, Distance yDistance) {
        return getTimeAtHoriAxis(candidate, yDistance, false);
    }

    /**
     * Get the time taken to reach a given X position (relative to the field)
     * for a given launch velocity
     *
     * @param candidate The launch velocity
     * @param xDistance The X position (relative to the field)
     * @return The time to the given position
     */
    public Time getTimeAtX(Velocity3d candidate, Distance xDistance) {
        return getTimeAtXRelative(candidate, xDistance.minus(posLaunch.getMeasureX()));
    }


    /**
     * Get the time taken to reach a given Y position (relative to the field)
     * for a given launch velocity
     *
     * @param candidate The launch velocity
     * @param yDistance The Y position (relative to the field)
     * @return The time to the given position
     */
    public Time getTimeAtY(Velocity3d candidate, Distance yDistance) {
        return getTimeAtYRelative(candidate, yDistance.minus(posLaunch.getMeasureY()));
    }


    /**
     * Get the time to target on a given launch velocity
     * 
     * @param candidate The launch velocity
     * @return The time to target
     */
    public Time getTimeAtTarget(Velocity3d candidate) {
        return this.getTimeAtX(candidate, posTarget.getMeasureX());
    }

}
