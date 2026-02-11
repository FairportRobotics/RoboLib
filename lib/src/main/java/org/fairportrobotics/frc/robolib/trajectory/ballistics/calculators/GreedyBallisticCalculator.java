package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import java.util.Arrays;
import java.util.LinkedList;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.BCEvalParams;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;

import edu.wpi.first.units.measure.Angle;

public class GreedyBallisticCalculator extends BallisticCalculator {
    public GreedyBallisticCalculator() {
        super();
    }


    public <M extends BallisticModel<P>, P> BCResult computeSolution(
        M           model,
        P           initParam,
        Velocity3d  robotVelocity,
        Angle       robotAngle,
        Velocity3d  shooterVelocity
    ) {
        if(initParam == null) {
            initParam = model.getInitParam();
        }

        LinkedList<P> candidate_params = new LinkedList<>();
        candidate_params.addFirst(initParam);

        while(!candidate_params.isEmpty()) {
            // Evaluate the candidates
            P           param       = candidate_params.removeFirst();
            Velocity3d  candidate   = model.getCandidateVelocity(param);

            if(candidate != null) {
                BCEvalParams    evalParams = new BCEvalParams(robotVelocity, robotAngle, shooterVelocity, candidate);
                Double          penalty = this.evaluateCandidate(model, evalParams);
                if(penalty != null) {
                    return new BCResult(penalty, evalParams);
                }
            }

            // Update the candidate list
            candidate_params.addAll(Arrays.asList(model.getNeighborParams(param)));
        }
        return new BCResult(null, null);
    }
}
