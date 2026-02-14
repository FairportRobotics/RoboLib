package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;

import java.util.ArrayList;
import java.util.HashSet;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class ConstantBM extends ReversableRadialBM<Integer> {
    private Velocity3d velocityResult;
    private Translation3d positionResult;
    private Time revResult;
    private int initParam;
    private int minParam;
    private int maxParam;

    private HashSet<Integer> computedParams;


    //
    //  Setup code
    //

    public ConstantBM(
        Translation3d posLaunch,
        Translation3d posTarget
    ) {
        super(posLaunch, posTarget);

        this.velocityResult = null;
        this.positionResult = null;
        this.revResult      = null;

        this.initParam      = 0;
        this.minParam       = 0;
        this.maxParam       = 0;
        this.computedParams = new HashSet<>();
    }


    public ConstantBM setVelocityResult(Velocity3d velocityResult) {
        this.velocityResult = velocityResult;
        return this;
    }


    public ConstantBM setPositionResult(Translation3d positionResult) {
        this.positionResult = positionResult;
        return this;
    }


    public ConstantBM setRevResult(Time revResult) {
        this.revResult = revResult;
        return this;
    }


    public ConstantBM setInitParam(int initParam) {
        this.initParam = initParam;
        return this;
    }


    public ConstantBM setMinParam(int minParam) {
        this.minParam = minParam;
        return this;
    }


    //
    //  Parameter manipulation
    //

    public Integer getInitParam() {
        return this.initParam;
    }


    public Integer[] getNeighborParams(Integer param) {
        ArrayList<Integer> neighbors = new ArrayList<>();
        if(param-1 >= minParam) neighbors.add(param-1);
        if(param+1 <= maxParam) neighbors.add(param+1);
        return neighbors.toArray(new Integer[0]);
    }


    //
    //  Velocity modelling
    //

    public Velocity3d getCandidateVelocity(Integer param) {
        this.computedParams.add(param);
        return this.velocityResult;
    }


    public Translation3d positionAtTime(Velocity3d velocity, Time time) {
        return this.positionResult;
    }


    //
    //  Reverse functions
    //

    public Time getTimeAtRadius(Velocity3d candidate, Distance radialDistance) {
        return revResult;
    }


    //
    //  Candidate tracking
    //

    public boolean paramComputed(Integer param) {
        return this.computedParams.contains(param);
    }


    public void paramComputedReset() {
        this.computedParams = new HashSet<>();
    }


}
