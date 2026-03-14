package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import java.util.Arrays;
import java.util.LinkedList;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions.BallisticException;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;

import edu.wpi.first.units.measure.Angle;

/**
 * Simple greedy calculator (i.e. returns the first valid result)
 */
public class GreedyBallisticCalculator extends BallisticCalculator {
    /**
     * Default constructor for the calculator
     */
    public GreedyBallisticCalculator() {
        super();
    }

    @Override
    public <M extends BallisticModel<P>, P> BCResult computeSolution(
        M           model,
        P           initParam,
        Velocity3d  robotVelocity,
        Angle       robotAngle,
        Velocity3d  shooterVelocity
    ) throws BallisticException {
        model.paramComputedReset();

        LinkedList<P> candidate_params = new LinkedList<>();
        candidate_params.addFirst(initParam);

        while(!candidate_params.isEmpty()) {
            // Evaluate the candidates
            P           param       = candidate_params.removeFirst();
            if(model.paramComputed(param)) continue;

            BCResult candidate = this.evaluateCandidate(
                model, param, robotVelocity, robotAngle, shooterVelocity
            );
            if(candidate.isValid()) {
                return candidate;
            }

            // Update the candidate list
            candidate_params.addAll(Arrays.asList(model.getNeighborParams(param)));
        }
        return BCResult.kNoSolution;
    }
}
