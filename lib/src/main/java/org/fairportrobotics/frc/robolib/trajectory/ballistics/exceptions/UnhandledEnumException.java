package org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions;

/**
 * Base exception for the ballistic calculator
 */
public class UnhandledEnumException extends BallisticException {
    public UnhandledEnumException() {
        super();
    }

    public UnhandledEnumException(String message) {
        super(message);
    }
}
