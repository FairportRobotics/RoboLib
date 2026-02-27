package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;

public class SimpleBM extends BallisticModel<Integer> {


    //
    //  Setup code
    //

    public SimpleBM() {
        this(Translation3d.kZero, Translation3d.kZero);
    }

    public SimpleBM(
        Translation3d posLaunch,
        Translation3d posTarget
    ) {
        super(posLaunch, posTarget);
    }


    //
    //  Parameter manipulation
    //

    public Integer getInitParam() {
        return 0;
    }


    public Integer[] getNeighborParams(Integer param) {
        return new Integer[0];
    }


    //
    //  Velocity modelling
    //

    public Velocity3d getCandidateVelocity(Integer param) {
        return Velocity3d.zero;
    }


    public Translation3d positionAtTime(Velocity3d velocity, Time time) {
        return Translation3d.kZero;
    }


    //
    //  Candidate tracking
    //

    public boolean paramComputed(Integer param) {
        return false;
    }


    public void paramComputedReset() {}


}
