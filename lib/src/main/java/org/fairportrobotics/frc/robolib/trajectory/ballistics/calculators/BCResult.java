package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.BCEvalParams;

public class BCResult {
    public final Double         penalty;    /// The penalty for the computed launcher velocity (null if invalid)
    public final BCEvalParams   evalParams; /// The BCEvalParams instance that produced the best penalty


    public BCResult(
        Double          penalty,
        BCEvalParams    evalParams
    ) {
        this.penalty    = penalty;
        this.evalParams = evalParams;
    }

    public boolean isValid() {
        return this.penalty != null;
    }
}
