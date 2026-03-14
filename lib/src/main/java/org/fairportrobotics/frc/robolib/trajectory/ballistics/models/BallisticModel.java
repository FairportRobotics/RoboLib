package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/// Base class for ballistic models
/// 
/// The model is responsible for:
///     - Tracking its launch and target positions (and relative stats)
///     - Managing 
public abstract class BallisticModel<P> {
    /// The launch position (i.e. where the projectile leaves the shooter)
    protected final Translation3d posLaunch;
    /// The target position (i.e. what we're trying to hit)
    protected final Translation3d posTarget;

    /// The position of posTarget relative to posLaunch
    protected final Translation3d targetPosRelative;
    /// The horizontal disance of targetPosRelative
    protected final Distance relativeHorizontalDistance;

    /**
     * Constructo which sets the model
     * @param posLaunch The inital posLaunch
     * @param posTarget The inital posTarget
     */
    public BallisticModel(Translation3d posLaunch, Translation3d posTarget) {
        this.posLaunch      = posLaunch;
        this.posTarget      = posTarget;

        this.targetPosRelative = this.posTarget.minus(this.posLaunch);
        this.relativeHorizontalDistance = Units.Meters.of(Math.hypot(
            this.targetPosRelative.getMeasureX().in(Units.Meters),
            this.targetPosRelative.getMeasureY().in(Units.Meters)
        ));
    }

    //
    //  Base getters
    //

    /**
     * Get the current launch position
     * @return The current launch position
     */
    public Translation3d getPosLaunch() {
        return this.posLaunch;
    }

    /**
     * Get the current target position
     * @return The current target position
     */
    public Translation3d getPosTarget() {
        return this.posTarget;
    }

    /**
     * Get the position of posTarget relative to posLaunch
     *
     * @return The position of posTarget relative to posLaunch
     */
    public Translation3d getTargetPosRelative() {
        return this.targetPosRelative;
    }

    /**
     * Get the horizontal distance between posLaunch and posTarget
     *
     * @return The horizontal distance between posLaunch and posTarget
     */
    public Distance getRelativeHorizontalDistance() {
        return this.relativeHorizontalDistance;
    }

    //
    //  Parameter Manipulation
    //

    /**
     * Returns a recommended starting parameter
     *
     * @return Returns a recommended starting parameter
     */
    public abstract P getInitParam();


    /**
     * Get parameters that are "adjacent" to the input paramter (i.e. generate
     * the next batch of candidates that (may) improve on this one)
     *
     * @param param The base parameter
     * @return An array containing all neighbor params
     */
    public abstract P[] getNeighborParams(P param);


    //
    //  Velocity modelling
    //

    /**
     * Get an initial velocity (in the same coordinate space as posLaunch and
     * posTarget) which will pass thru posLaunch and posTarget with the given
     * model configurations and parameters.
     *
     * @param param The parameters used to determine this candidate
     * @return The computed result for the given parameter
     */
    public abstract Velocity3d getCandidateVelocity(P param);


    /**
     * Get the position of the projectile at a given time offset for a given
     * starting velocity
     *
     * @param velocity The velocity to check
     * @param time The time to check
     * @return The expected position at the given time
     */
    public abstract Translation3d positionAtTime(Velocity3d velocity, Time time);

    //
    //  Computed parameter check
    //

    /**
     * Check if a given parameter has had a computed velocity.
     *
     * @param param The param to check
     * @return true iff the parameter has had a computed solution
     */
    public abstract boolean paramComputed(P param);

    /**
     * Mark all params as not computed for paramComputed
     */
    public abstract void paramComputedReset();
}
