package org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions;

/**
 * Base exception for the ballistic calculator
 */
public class BallisticException extends Exception {
    public BallisticException() {
        super();
    }

    public BallisticException(String message) {
        super(message);
    }
}
