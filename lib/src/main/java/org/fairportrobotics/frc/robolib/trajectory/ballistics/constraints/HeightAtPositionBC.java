package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import org.fairportrobotics.frc.robolib.Util;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions.UnhandledEnumException;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.ReversableRadialBM;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * Ballistic constraint requiring ball be a minimum height at a given
 * radius from the target
 */
public class HeightAtPositionBC extends BallisticConstraint {

    /**
     * The types of position to evaluate forf
     */
    public enum PositionType {
        /** Evaluate with the radial distance away from the target */
        TargetRadius,
        /** Evaluate against an absolute X position */
        AbsoluteX,
        /** Evaluate against an absolute Y position */
        AbsoluteY,
        /** Evaluate against an X position relative to launch position */
        RelativeX,
        /** Evaluate against a Y position relative to the launch position */
        RelativeY
    };

    private final double minHeightMeters;
    private final double safeHeightMeters;
    private final Distance testPosition;
    private final PositionType positionType;

    /**
     * Main constructor
     *
     * @param minHeight The minimum passing height
     * @param safeHeight The height above which no penalty is applied
     * @param testPosition The position to test at
     * @param positionType The type of position check to run
     */
    public HeightAtPositionBC(
        Distance minHeight, Distance safeHeight, Distance testPosition, PositionType positionType
    ) {
        this.minHeightMeters    = minHeight.in(Units.Meters);
        this.safeHeightMeters   = safeHeight.in(Units.Meters);
        this.testPosition       = testPosition;
        this.positionType       = positionType;
    }


    /**
     * Evaluation function for a reversable, radial ballistic model
     *
     * @param model The model to evaluate against
     * @param evalParams The params to evaluate
     * @return The penalty percentage (from 0 to 1, inclusive). null for constraint failure
     * @throws UnhandledEnumException If this.positionType is not valid
     */
    private Double evaluate_reversable(
        ReversableRadialBM<?> model, BCEvalParams evalParams
    ) throws UnhandledEnumException{
        Time timeAtTarget;
        switch(this.positionType) {
            case TargetRadius:
                timeAtTarget = model.getTimeAtRadius(
                    evalParams.getCandidateVelocity(),
                    model.getRelativeHorizontalDistance().minus(testPosition)
                );
                break;
            case AbsoluteX:
                timeAtTarget = model.getTimeAtX(evalParams.getCandidateVelocity(), testPosition);
                break;
            case AbsoluteY:
                timeAtTarget = model.getTimeAtY(evalParams.getCandidateVelocity(), testPosition);
                break;
            case RelativeX:
                timeAtTarget = model.getTimeAtXRelative(evalParams.getCandidateVelocity(), testPosition);
                break;
            case RelativeY:
                timeAtTarget = model.getTimeAtYRelative(evalParams.getCandidateVelocity(), testPosition);
                break;
            default:
                throw new UnhandledEnumException(this.positionType.name());
        }
        double zAtTargetMeters = model.positionAtTime(
            evalParams.getCandidateVelocity(),
            timeAtTarget
        ).getMeasureZ().in(Units.Meters);

        if(zAtTargetMeters > safeHeightMeters) {
            return 0.0;
        } else if(zAtTargetMeters < minHeightMeters) {
            return null;
        } else {
            return Util.rangePercent(minHeightMeters, safeHeightMeters, zAtTargetMeters);
        }
    }

    /**
     * General evaluation function
     *
     * @param <M> The type of model to evaluate for
     * @param <P> The type of model parameter to evaluate for
     * @param model The model parameter to evaluate for
     * @param evalParams The params to evaluate
     * @return The penalty percentage (from 0 to 1, inclusive). null for constraint failure
     * @throws UnhandledEnumException If this.positionType is not valid
     */
    public <M extends BallisticModel<P>, P> Double evaluate(
        M model, BCEvalParams evalParams
    ) throws UnhandledEnumException {
        if(model instanceof ReversableRadialBM) {
            return this.evaluate_reversable((ReversableRadialBM<?>) model, evalParams);
        }
        return null;
    }
}
