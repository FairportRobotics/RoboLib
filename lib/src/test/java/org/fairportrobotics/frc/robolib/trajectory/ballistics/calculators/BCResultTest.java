package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class BCResultTest {
    @Test static void testIsValid() {
        assertTrue(new BCResult(1.0, null).isValid(), "Valid penalty");
        assertFalse(new BCResult(null, null).isValid(), "Invalid penalty");
    }
}
