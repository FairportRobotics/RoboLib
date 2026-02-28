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

    enum PositionType {
        TargetRadius,
        AbsoluteX,
        AbsoluteY,
        RelativeX,
        RelativeY
    };

    private final double minHeightMeters;
    private final double safeHeightMeters;
    private final Distance testPosition;
    private final PositionType positionType;

    /**
     * @param minHeight The minimum passing height
     * @param safeHeight The height above which no penalty is applied
     * @param testPosition The position to test at
     * @param positionType The type of position check to run
     */
    public HeightAtPositionBC(
        Distance minHeight, Distance safeHeight, Distance testRadius, PositionType positionType
    ) {
        this.minHeightMeters    = minHeight.in(Units.Meters);
        this.safeHeightMeters   = safeHeight.in(Units.Meters);
        this.testPosition       = testRadius;
        this.positionType       = positionType;
    }


    public <M extends ReversableRadialBM<?>> Double evaluate_reversable(
        M model, BCEvalParams evalParams
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


    public <M extends BallisticModel<P>, P> Double evaluate(
        M model, BCEvalParams evalParams
    ) throws UnhandledEnumException {
        if(model instanceof ReversableRadialBM) {
            return this.evaluate_reversable((ReversableRadialBM<?>) model, evalParams);
        }
        return null;
    }
}
