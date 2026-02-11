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

    /**
     * @return Returns a recommended starting parameter
     */
    public abstract P getInitParam();

    /**
     * Get an abstract velocity (in the same coordinate space as posLaunch and
     * posTarget) which will pass thru
     *
     * @param param The parameters used to determine this candidate
     * @return The computed result for the given parameter
     */
    public abstract Velocity3d getCandidateVelocity(P param);

    /**
     * Get parameters that are "adjacent" to the input paramter (i.e. generate
     * the next batch of candidates that (may) improve on this one)
     *
     * @param param The base parameter
     * @return An array containing all neighbor params
     */
    public abstract P[] getNeighborParams(P param);

    /**
     * Get the position of the projectile at a given time offset
     *
     * @param velocity The velocity to check
     * @param time The time to check
     * @return The expected position at the given time
     */
    public abstract Translation3d positionAtTime(Velocity3d velocity, Time time);

}
