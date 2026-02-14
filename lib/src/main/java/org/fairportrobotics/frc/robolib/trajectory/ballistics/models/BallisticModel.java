package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;

public abstract class BallisticModel<P> {
    protected Translation3d posLaunch;
    protected Translation3d posTarget;

    public BallisticModel(Translation3d posLaunch, Translation3d posTarget) {
        this.posLaunch      = posLaunch;
        this.posTarget      = posTarget;
    }

    //
    //  Base getters
    //

    public Translation3d getPosLaunch() {
        return this.posLaunch;
    }

    public Translation3d getPosTarget() {
        return this.posTarget;
    }

    //
    //  Parameter Manipulation
    //

    /**
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
     * model configurations and parameters
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
     * Check if a given parameter has had a computed velocity
     * @param param The param to check
     * @return true iff the parameter has had a computed solution
     */
    public abstract boolean paramComputed(P param);

    /**
     * Mark all params as not computed for paramComputed
     */
    public abstract void paramComputedReset();
}
