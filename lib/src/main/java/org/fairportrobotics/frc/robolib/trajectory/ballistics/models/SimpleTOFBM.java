package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;

import java.util.ArrayList;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;


public class SimpleTOFBM extends ReversableRadialBM<Time> {

    private final LinearAcceleration k_gravity = Units.MetersPerSecondPerSecond.of(-9.81);

    protected final Time tofMin;
    protected final Time tofMax;
    protected final Time tofStep;

    protected final Translation3d   targetPosRelative;
    protected final Distance        relativeHorizontalDistance;
    protected final Angle           azimuth;

    public SimpleTOFBM(
        Translation3d posLaunch, Translation3d posTarget,
        Time tofMin, Time tofMax, Time tofStep
    ) {
        super(posLaunch, posTarget);

        // Get time of flight params
        this.tofMin     = tofMin;
        this.tofMax     = tofMax;
        this.tofStep    = tofStep;

        // Pre-compute components
        this.targetPosRelative = this.posTarget.minus(this.posLaunch);
        this.relativeHorizontalDistance = Units.Meters.of(
            Math.hypot(
                this.targetPosRelative.getMeasureX().in(Units.Meters),
                this.targetPosRelative.getMeasureY().in(Units.Meters)
            )
        );
        this.azimuth = Units.Radians.of(
            Math.atan2(targetPosRelative.getY(), targetPosRelative.getX()
        ));
    }


    public Time getInitParam() {
        return this.tofMin.plus(this.tofMax).div(2);
    }


    public Velocity3d getCandidateVelocity(Time param) {
        return new Velocity3d(
            this.relativeHorizontalDistance.div(param),
            this.targetPosRelative.getMeasureZ().div(param)
                .plus(k_gravity.times(param).times(0.5)),
            azimuth
        );
    }


    public Time[] getNeighborParams(Time param) {
        ArrayList<Time> neighbors = new ArrayList<>();

        Time            lower  = param.minus(tofStep);
        if(lower.gte(tofMin))   neighbors.add(lower);

        Time            higher = param.plus(tofStep);
        if(higher.lte(tofMax))  neighbors.add(higher);

        return neighbors.toArray(new Time[0]);
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


    public Time getTimeAtRadius(Velocity3d candidate, Distance radialDistance) {
        return radialDistance.div(candidate.getHorizontalVelocity());
    }
}
