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
     * Get the time taken to travel a given X distance (relative to posLaunch) on a given candidate
     * trajectory
     *
     * @param candidate The generated velocity to compute against
     * @param xDistance The X distance relative to posLaunch to solve for
     * @return The time it takes to travel a given X distance relative to posTarget
     */
    public Time getTimeAtX(Velocity3d candidate, Distance xDistance) {
        Distance radialDistance = Units.Meters.of(
            xDistance.minus(posLaunch.getMeasureX()).in(Units.Meters)
                / Math.cos(candidate.getAzimuthAngle().in(Units.Radians))
        );
        return this.getTimeAtRadius(candidate, radialDistance);
    }


    /**
     * Get the time taken to travel a given Y distance (relative to posLaunch) on a given candidate
     * trajectory
     *
     * @param candidate The generated velocity to compute against
     * @param yDistance The Y distance relative to posLaunch to solve for
     * @return The time it takes to travel a given Y distance relative to posTarget
     */
    public Time getTimeAtY(Velocity3d candidate, Distance yDistance) {
        Distance radialDistance = Units.Meters.of(
            yDistance.minus(posLaunch.getMeasureY()).in(Units.Meters)
                / Math.sin(candidate.getAzimuthAngle().in(Units.Radians))
        );
        return this.getTimeAtRadius(candidate, radialDistance);
    }


    /**
     * Get the time to target on a given candidate trajectory
     * 
     * @param candidate The generated velocity to compute against
     * @return The time it takes to travel to the target position
     */
    public Time getTimeAtTarget(Velocity3d candidate) {
        return this.getTimeAtX(candidate, posTarget.getMeasureX());
    }
}
