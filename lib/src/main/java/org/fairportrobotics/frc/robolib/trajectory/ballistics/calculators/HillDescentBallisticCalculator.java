package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.BCEvalParams;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions.BallisticException;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;

import edu.wpi.first.units.measure.Angle;

public class HillDescentBallisticCalculator extends BallisticCalculator {
    public HillDescentBallisticCalculator() {
        super();
    }


    private <M extends BallisticModel<P>, P> BCResult evalSingleCandidate(
        M           model,
        P           modelParam,
        Velocity3d  robotVelocity,
        Angle       robotAngle,
        Velocity3d  shooterVelocity
    ) throws BallisticException {
        Velocity3d candidateVelocity    = model.getCandidateVelocity(modelParam);
        BCEvalParams evalParams         = new BCEvalParams(robotVelocity, robotAngle, shooterVelocity, candidateVelocity);
        return new BCResult(this.evaluateCandidate(model, evalParams), evalParams);
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
        BCResult    bestResult          = this.evalSingleCandidate(model, initParam, robotVelocity, robotAngle, shooterVelocity);
        boolean     foundImprovement    = true;

        while(bestResult.isValid() && foundImprovement) {
            // Evaluate all neighbors of the current param
            foundImprovement = false;
            for (P param : model.getNeighborParams(bestParam)) {
                if(model.paramComputed(param)) continue;

                BCResult result = this.evalSingleCandidate(model, param, robotVelocity, robotAngle, shooterVelocity);
                if(result.penalty < bestResult.penalty) {
                    bestParam           = param;
                    bestResult          = result;
                    foundImprovement    = true;
                }
            }
        }
        return bestResult;
    }
}
