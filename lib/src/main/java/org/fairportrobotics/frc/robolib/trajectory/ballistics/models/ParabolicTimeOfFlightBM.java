package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;

import java.util.ArrayList;
import java.util.HashSet;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;


/**
 * Parabolic ballistic model, with solutions computed from time of flight
 */
public class ParabolicTimeOfFlightBM extends ReversableRadialBM<Time> {
    /** The linear acceleration of gravity in the +Z axis (positive is upwards) */
    public static final LinearAcceleration k_gravity = Units.MetersPerSecondPerSecond.of(-9.81);

    /** The minimum time of flight to consider */
    protected final Time    tofMin;
    /** The maximum time of flight to consider */
    protected final Time    tofMax;
    /** The step between parameters */
    protected final Time    tofStep;

    /** The azimuth angle between posLaunch and posTarget */
    protected final Angle   azimuth;

    /** The times (in seconds) that have been computed for */
    private HashSet<Double> computedSeconds;

    /**
     * Main constructor
     * 
     * @param posLaunch The position projectiles are launched from
     * @param posTarget The target position for projectiles
     * @param tofMin The minimum time of flight to consider
     * @param tofMax The maximum time of flight to consider
     * @param tofStep The timestep between candidate times of flight
     */
    public ParabolicTimeOfFlightBM(
        Translation3d posLaunch, Translation3d posTarget,
        Time tofMin, Time tofMax, Time tofStep
    ) {
        super(posLaunch, posTarget);

        // Get time of flight params
        this.tofMin     = tofMin;
        this.tofMax     = tofMax;
        this.tofStep    = tofStep;

        // Pre-compute components
        this.azimuth = Units.Radians.of(
            Math.atan2(targetPosRelative.getY(), targetPosRelative.getX()
        ));

        // Init the seen param tracking
        this.computedSeconds = new HashSet<>();
    }

    //
    //  Base getters
    //

    /**
     * Gets the minimum time of flight
     * @return the minimum time of flight
     */
    public Time getTOFMin() {
        return this.tofMin;
    }


    /**
     * Get the maximum time of flight
     * @return The maximum time of flight
     */
    public Time getTOFMax() {
        return this.tofMax;
    }


    /**
     * Get the timestep between candidate times of flight
     *
     * @return The timestep between candidate times of flight
     */
    public Time getTOFStep() {
        return this.tofStep;
    }


    /**
     * Get the computed azimuth angle from the launch position to the target position
     *
     * @return the computed azimuth angle from the launch position to the target position
     */
    public Angle getAzimuth() {
        return this.azimuth;
    }


    //
    //  Parameter manipulation
    //

    public Time getInitParam() {
        return this.tofMin.plus(this.tofMax).div(2);
    }


    public Time[] getNeighborParams(Time param) {
        ArrayList<Time> neighbors = new ArrayList<>();

        Time            lower  = param.minus(tofStep);
        if(lower.gte(tofMin))   neighbors.add(lower);

        Time            higher = param.plus(tofStep);
        if(higher.lte(tofMax))  neighbors.add(higher);

        return neighbors.toArray(new Time[0]);
    }


    //
    //  Velocity modelling
    //

    public Velocity3d getCandidateVelocity(Time param) {
        this.computedSeconds.add(param.in(Units.Seconds));
        return new Velocity3d(
            this.relativeHorizontalDistance.div(param),
            this.targetPosRelative.getMeasureZ().div(param)
                .minus(k_gravity.times(param).times(0.5)),
            azimuth
        );
    }


    public Translation3d positionAtTime(Velocity3d velocity, Time time) {
        return new Translation3d(
            this.posLaunch.getMeasureX()
                .plus(velocity.getXVelocity().times(time)),
            this.posLaunch.getMeasureY()
                .plus(velocity.getYVelocity().times(time)),
            this.posLaunch.getMeasureZ()
                .plus(velocity.getZVelocity().times(time))
                .plus(k_gravity.times(time).times(time).times(0.5))
        );
    }


    //
    //  Reverse functions
    //

    public Time getTimeAtRadius(Velocity3d candidate, Distance radialDistance) {
        return radialDistance.div(candidate.getHorizontalVelocity());
    }

    //
    //  Computed parameter check
    //

    public boolean paramComputed(Time param) {
        return this.computedSeconds.contains(param.in(Units.Seconds));
    }


    public void paramComputedReset() {
        this.computedSeconds = new HashSet<>();
    }

}
