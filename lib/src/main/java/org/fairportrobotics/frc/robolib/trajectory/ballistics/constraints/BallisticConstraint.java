package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions.BallisticException;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;

public abstract class BallisticConstraint {
    protected double weight;

    /**
     * Generate the constraint with weight 0.0
     */
    public BallisticConstraint() {
        this(0.0);
    }

    /**
     * Generate the constraint with a given weight value
     * @param weight The weight of this constraint's penalty
     */
    public BallisticConstraint(double weight) {
        this.weight = weight;
    }

    /**
     * Set the weight of this constraint
     * @param weight The weight of the constraint
     * @return this
     */
    public BallisticConstraint setWeight(double weight) {
        this.weight = weight;
        return this;
    }

    /**
     * @return The weight of this constraint
     */
    public double getWeight() {
        return this.weight;
    }


    /**
     * Evaluate a given solution's fitness against this constraint
     *
     * @param <M> The type of BallisticModel
     * @param <P> The parameter type of M
     * @param model The model used to compute the candidate
     * @param evalParams The candidate parameters used in evaluation
     * @return A penalty value (from 0 to 1.0) for this match (null if candidate
     *  is invalid). Used with weight to determine candidate fitness
     */
    public abstract <M extends BallisticModel<P>, P> Double evaluate(
        M model, BCEvalParams evalParams
    ) throws BallisticException;
}
