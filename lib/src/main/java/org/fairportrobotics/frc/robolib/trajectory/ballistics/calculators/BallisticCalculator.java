package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import java.util.ArrayList;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.BCEvalParams;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.BallisticConstraint;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions.BallisticException;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;

import edu.wpi.first.units.measure.Angle;

public abstract class BallisticCalculator {
    private ArrayList<BallisticConstraint> constraints;
    private double velocityDiffWeight;

    //
    //  Setup code
    //

    public BallisticCalculator() {
        constraints = new ArrayList<>();
        this.velocityDiffWeight = 1.0;
    }

    public BallisticCalculator addConstraint(BallisticConstraint constraint) {
        constraints.add(constraint);
        return this;
    }

    public BallisticCalculator setVelocityDiffWeight(double weight) {
        this.velocityDiffWeight = weight;
        return this;
    }

    //
    //  Computation
    //

    /**
     * Compute the best solution for given initial conditions (pulling parameter
     * from model.getInitParam())
     *
     * @param <M> The type of model to compute against
     * @param <P> The candidate parameter type of the model
     * @param model The model to compute against
     * @param robotVelocity The velocity of the robot (in field coordinates)
     * @param robotAngle The robot's current heading (in field coordinates)
     * @param shooterVelocity The shooter's current launch velocity (in field coordinates)
     * @return The generated BCResult
     */
    public <M extends BallisticModel<P>, P> BCResult computeSolution(
        M           model,
        Velocity3d  robotVelocity,
        Angle       robotAngle,
        Velocity3d  shooterVelocity
    ) throws BallisticException {
        return this.computeSolution(
            model, model.getInitParam(), robotVelocity, robotAngle, shooterVelocity
        );
    }


    /**
     * Compute the best solution for given initial conditions
     *
     * @param <M> The type of model to compute against
     * @param <P> The candidate parameter type of the model
     * @param model The model to compute against
     * @param initParam The seed parameter (to start computation)
     * @param robotVelocity The velocity of the robot (in field coordinates)
     * @param robotAngle The robot's current heading (in field coordinates)
     * @param shooterVelocity The shooter's current launch velocity (in field coordinates)
     * @return The generated BCResult
     */
    public abstract <M extends BallisticModel<P>, P> BCResult computeSolution(
        M           model,
        P           initParam,
        Velocity3d  robotVelocity,
        Angle       robotAngle,
        Velocity3d  shooterVelocity
    ) throws BallisticException;


    /**
     * Get the average penalty for a given set of params
     *
     * @param <M> The type of model to compute against
     * @param <P> The candidate parameter type of the model
     * @param model The model to compute against
     * @param evalParams The eval function param block
     * @return The weighted average of penalties from velocity diferences and
     *  constraints (null if evalParams fail any constraints)
     */
    protected <M extends BallisticModel<P>, P> Double evaluateCandidate(
        M model, BCEvalParams evalParams
    ) throws BallisticException {
        // Get the weight component for velocity difference
        Velocity3d velShooterDiff =
            evalParams.getCandidateShooterVelocity().minus(evalParams.getCurrentShooterVelocity());

        double numerator    = velShooterDiff.getSquareMagnitude() * this.velocityDiffWeight;
        double denominator  = this.velocityDiffWeight;

        // Evaluate against all constraints
        for (BallisticConstraint constraint : this.constraints) {
            Double penaltyPercent = constraint.evaluate(model, evalParams);
            if(penaltyPercent == null) return null;

            numerator   += penaltyPercent * constraint.getWeight();
            denominator += constraint.getWeight();
        }

        // Get the final sum
        return numerator/denominator;
    }
}
