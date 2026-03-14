package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.BCEvalParams;

/**
 * Record type for ballistic calculator results
 */
public record BCResult(Double penalty, BCEvalParams evalParams) {

    /**
     * Checks if the result is valid (i.e. checks if penalty != null)
     * @return True if the evalParams evaluates as valid, false otherwise
     */
    public boolean isValid() {
        return this.penalty != null;
    }
}
