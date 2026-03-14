package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions.BallisticException;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;

import edu.wpi.first.units.measure.Angle;

/**
 * Solution calculator implementing a gradient descent algorithm to find a
 * solution with a local minimum penalty (not guaranteed to find a global
 * minimum penalty)
 */
public class HillDescentBallisticCalculator extends BallisticCalculator {

    /**
     * The default consttructor for the calculator
     */
    public HillDescentBallisticCalculator() {
        super();
    }

    public <M extends BallisticModel<P>, P> BCResult computeSolution(
        M           model,
        P           initParam,
        Velocity3d  robotVelocity,
        Angle       robotAngle,
        Velocity3d  shooterVelocity
    ) throws BallisticException {
        model.paramComputedReset();

        P           bestParam           = initParam;
        BCResult    bestResult          = this.evaluateCandidate(model, initParam, robotVelocity, robotAngle, shooterVelocity);
        boolean     foundImprovement    = true;

        while(bestResult.isValid() && foundImprovement) {
            // Evaluate all neighbors of the current param
            foundImprovement = false;
            for (P param : model.getNeighborParams(bestParam)) {
                if(model.paramComputed(param)) continue;

                BCResult result = this.evaluateCandidate(model, param, robotVelocity, robotAngle, shooterVelocity);
                if(result.isValid() && result.penalty() < bestResult.penalty()) {
                    bestParam           = param;
                    bestResult          = result;
                    foundImprovement    = true;
                }
            }
        }
        return bestResult;
    }
}
