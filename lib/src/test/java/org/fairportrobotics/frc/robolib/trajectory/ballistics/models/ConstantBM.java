package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class ConstantBM extends ReversableRadialBM<Integer> {

    //
    //  Internal classes
    //

    public record PositionAtTimeArgs(Velocity3d velocity, Time time) {};
    public record GetTimeAtPosArgs(Velocity3d candidate, Distance radialDistance) {};


    //
    //  Fields
    //

    private Velocity3d    velocityResult;
    private Translation3d positionResult;
    private Time          revResult;

    private int           initParam;
    private int           minParam;
    private int           maxParam;

    private HashSet<Integer> computedParams;

    public List<Integer>            getNeighborParamsArgs;
    public List<Integer>            getCandidateVelocityArgs;
    public List<PositionAtTimeArgs> getPositionAtTimeArgs;
    public List<GetTimeAtPosArgs>   getTimeAtRadiusArgs;
    public List<GetTimeAtPosArgs>   getTimeAtXRelativeArgs;
    public List<GetTimeAtPosArgs>   getTimeAtYRelativeArgs;
    public List<GetTimeAtPosArgs>   getTimeAtXArgs;
    public List<GetTimeAtPosArgs>   getTimeAtYArgs;
    public List<Velocity3d>         getTimeAtTargetArgs;


    public List<Integer> paramComputedArgs;

    //
    //  Setup code
    //

    public ConstantBM() {
        this(Translation3d.kZero, Translation3d.kZero);
    }

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

        this.getNeighborParamsArgs      = new LinkedList<>();
        this.getCandidateVelocityArgs   = new LinkedList<>();
        this.getPositionAtTimeArgs      = new LinkedList<>();
        this.getTimeAtRadiusArgs        = new LinkedList<>();
        this.getTimeAtXRelativeArgs     = new LinkedList<>();
        this.getTimeAtYRelativeArgs     = new LinkedList<>();
        this.getTimeAtXArgs             = new LinkedList<>();
        this.getTimeAtYArgs             = new LinkedList<>();
        this.getTimeAtTargetArgs        = new LinkedList<>();
    }

    public ConstantBM setVelocityResult(Velocity3d velocityResult) {
        this.velocityResult = velocityResult;
        return this;
    }

    public ConstantBM setPositionResult(Distance x, Distance y, Distance z) {
        return this.setPositionResult(new Translation3d(x, y, z));
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

    public ConstantBM setMaxParam(int maxParam) {
        this.maxParam = maxParam;
        return this;
    }

    //
    //  Parameter manipulation
    //

    public Integer getInitParam() {
        return this.initParam;
    }


    public Integer[] getNeighborParams(Integer param) {
        this.getNeighborParamsArgs.add(param);
        ArrayList<Integer> neighbors = new ArrayList<>();
        if(param-1 >= minParam) neighbors.add(param-1);
        if(param+1 <= maxParam) neighbors.add(param+1);
        return neighbors.toArray(new Integer[0]);
    }


    //
    //  Velocity modelling
    //

    public Velocity3d getCandidateVelocity(Integer param) {
        this.getCandidateVelocityArgs.add(param);
        this.computedParams.add(param);
        return this.velocityResult;
    }


    public Translation3d positionAtTime(Velocity3d velocity, Time time) {
        this.getPositionAtTimeArgs.add(new PositionAtTimeArgs(velocity, time));
        return this.positionResult;
    }


    //
    //  Reverse functions
    //

    public Time getTimeAtRadius(Velocity3d candidate, Distance radialDistance) {
        this.getTimeAtRadiusArgs.add(new GetTimeAtPosArgs(candidate, radialDistance));
        return revResult;
    }

    public Time getTimeAtXRelative(Velocity3d candidate, Distance xDistance) {
        this.getTimeAtXRelativeArgs.add(new GetTimeAtPosArgs(candidate, xDistance));
        return super.getTimeAtXRelative(candidate, xDistance);
    }

    public Time getTimeAtYRelative(Velocity3d candidate, Distance yDistance) {
        this.getTimeAtYRelativeArgs.add(new GetTimeAtPosArgs(candidate, yDistance));
        return super.getTimeAtYRelative(candidate, yDistance);
    }

    public Time getTimeAtX(Velocity3d candidate, Distance xDistance) {
        this.getTimeAtXArgs.add(new GetTimeAtPosArgs(candidate, xDistance));
        return super.getTimeAtXRelative(candidate, xDistance);
    }

    public Time getTimeAtY(Velocity3d candidate, Distance yDistance) {
        this.getTimeAtYArgs.add(new GetTimeAtPosArgs(candidate, yDistance));
        return super.getTimeAtY(candidate, yDistance);
    }


    //
    //  Candidate tracking
    //

    public boolean paramComputed(Integer param) {
        this.paramComputedArgs.add(param);
        return this.computedParams.contains(param);
    }


    public void paramComputedReset() {
        this.computedParams = new HashSet<>();
    }


}
