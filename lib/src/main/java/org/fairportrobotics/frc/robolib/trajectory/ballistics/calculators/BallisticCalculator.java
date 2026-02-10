package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import java.util.ArrayList;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.BCEvalParams;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.BallisticConstraint;
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
     * Public solution
     * @param <M> The type of model to compute against
     * @param <P> The candidate parameter type of the model
     * @param model The model to compute against
     * @param initParam The seed parameter (to start computation)
     * @param velRobot The velocity of the robot (in the model's coordinate system)
     * @param velShooter The velocity currently imparted by the shooter (in the model's coordinate system)
     * @param robotHeading The heading of the robot (in the model's coordinate system)
     * @return The generated BCResult
     */
    public abstract <M extends BallisticModel<P>, P> BCResult<P> computeSolution(
        M model, Velocity3d velRobot, Velocity3d velShooter,
        Angle robotHeading, boolean quickMode
    );


    protected <M extends BallisticModel<P>, P> Double evaluateCandidate(
        M model, BCEvalParams evalParams
    ) {
        // Get the weight component for velocity difference
        Velocity3d velShooterDiff =
            evalParams.getCandidateGlobalShooterVelocity()
            .minus(evalParams.getCurrentGlobalShooterVelocity());
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
