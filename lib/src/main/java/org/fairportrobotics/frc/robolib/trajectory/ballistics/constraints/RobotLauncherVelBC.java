package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import org.fairportrobotics.frc.robolib.Util;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * A ballistic constraint defining the minimum and maximum values for
 * robot-relative launcher velocity (in spherical coordinates)
 */
public class RobotLauncherVelBC extends BallisticConstraint {
    private final Double    minSpeedMPS;
    private final Double    maxSpeedMPS;
    private final Double    targetSpeedPercent;

    private final Double    minAzimuthDegrees;
    private final Double    maxAzimuthDegrees;
    private final Double    targetAzimuthPercent;

    private final Double    minElevationDegrees;
    private final Double    maxElevationDegrees;
    private final Double    targetElevationPercent;

    class BoundViolated extends Exception {}

    /**
     * <p>Full constructor for the constraint</p>
     *
     * <p>Percent variables are a percentage (0.0-1.0) of the range between the
     * min and max values for the given value where no penalty is applied. Range
     * is always centered on the midpoint of the values</p>
     * 
     * <p>min, max, and target percent values must all be non-null to allow
     * evaluation on component. If any component is null, penalty will not be
     * considered (pass-fail check will still be performed). If no component
     * provides a penalty, 0.0 will be returned by default.</p>
     *
     * @param minSpeed Minimum radial velocity (nullable)
     * @param maxSpeed Maximum radial velocity (nullable)
     * @param targetSpeedPercent Ideal percent range for radial velocity (nullable)
     * @param minAzimuth Minimum azimuth angle (nullable)
     * @param maxAzimuth Maximum azimuth angle (nullable)
     * @param targetAzimuthPercent Ideal percent range for azimuth (nullable)
     * @param minElevation Minimum elevation angle (nullable)
     * @param maxElevation Maximum elevation angle (nullable)
     * @param targetElevationPercent Ideal percent range for elevation (nullable)
     */
    public RobotLauncherVelBC(
        LinearVelocity minSpeed, LinearVelocity maxSpeed, Double targetSpeedPercent,
        Angle minAzimuth, Angle maxAzimuth, Double targetAzimuthPercent,
        Angle minElevation, Angle maxElevation, Double targetElevationPercent
    ) {
        this.minSpeedMPS            = minSpeed.in(Units.MetersPerSecond);
        this.maxSpeedMPS            = maxSpeed.in(Units.MetersPerSecond);
        this.targetSpeedPercent     = targetSpeedPercent;

        this.minAzimuthDegrees      = minAzimuth.in(Units.Degrees);
        this.maxAzimuthDegrees      = maxAzimuth.in(Units.Degrees);
        this.targetAzimuthPercent   = targetAzimuthPercent;

        this.minElevationDegrees    = minElevation.in(Units.Degrees);
        this.maxElevationDegrees    = maxElevation.in(Units.Degrees);
        this.targetElevationPercent = targetElevationPercent;
    }


    /**
     * Get the weight contribution of a single component
     *
     * @param lowerBound The lower bound value to compute for
     * @param upperBound The upper bound value to compute for
     * @param idealPercentRange The ideal (0 penalty) percent range
     * @param value The value to check for
     * @return The partial penalty value (from 0 to 1, inclusive, null if unable
     * to compute penalty)
     * @throws BoundViolated Thrown if the constraint is violated
     */
    private Double getPartialWeight(
        Double lowerBound, Double upperBound, Double idealPercentRange, double value
    ) throws BoundViolated {
        // There is nothing to check for this bound
        if(lowerBound == null && upperBound == null) {
            return null;
        }

        // Perform bounds checking
        if(
            (lowerBound != null && value < lowerBound) || 
            (upperBound != null && value > upperBound)
        ) {
            throw new BoundViolated();
        } else if(lowerBound == null || upperBound == null || idealPercentRange == null) {
            return null;
        }

        double idealMin     = 0.5 - idealPercentRange/2;
        double idealMax     = 0.5 + idealPercentRange/2;
        double valuePercent = Util.rangePercent(lowerBound, upperBound, value); 

        if(valuePercent < idealMin) {
            return Util.rangePercent(idealMin, 0.0, valuePercent);
        } else if(valuePercent > idealMax) {
            return Util.rangePercent(idealMax, 1.0, valuePercent);
        } else {
            return 0.0;
        }
    }

    /**
     * Evaluate the constraint.
     * 
     * @param <M> The type of model to evaluate for
     * @param <P> The type of parameter to evaluate for
     * @param model The model to evaluate with
     * @param evalParams The evaluation params
     * @return The max penalty from each of the speed, azimuth, and elevation
     * components and 0.0 (null if constraint violated).
     */
    public <M extends BallisticModel<P>, P> Double evaluate(
        M model, BCEvalParams evalParams
    ) {
        Double speedWeight   = null;
        Double azimuthWeight    = null;
        Double elevationWeight  = null;

        // Compute the partial penalty for velocity
        try {
            speedWeight = this.getPartialWeight(
                minSpeedMPS, maxSpeedMPS, targetSpeedPercent,
                evalParams.getCandidateShooterVelocityRelative().getSpeed().in(Units.MetersPerSecond)
            );
            azimuthWeight = this.getPartialWeight(
                minAzimuthDegrees, maxAzimuthDegrees, targetAzimuthPercent,
                evalParams.getCandidateShooterVelocityRelative().getAzimuthAngle().in(Units.Degrees)
            );
            elevationWeight = this.getPartialWeight(
                minElevationDegrees, maxElevationDegrees, targetElevationPercent,
                evalParams.getCandidateShooterVelocityRelative().getElevationAngle().in(Units.Degrees)
            );
        } catch(BoundViolated e) {
            return null;
        }

        double maxWeight = 0.0;
        for (Double partialWeight : new Double[]{speedWeight, azimuthWeight, elevationWeight}) {
            if(partialWeight != null) {
                maxWeight = Math.max(partialWeight, maxWeight);
            }
        }
        return maxWeight;
    }
}
