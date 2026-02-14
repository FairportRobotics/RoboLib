package org.fairportrobotics.frc.robolib;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class UtilTest {
    @Test void testRangePercent() {
        assertEquals(
            0.2,
            Util.rangePercent(0, 10, 2),
            "Positives, Normal"
        );
        assertEquals(
            0.8,
            Util.rangePercent(10, 0, 2),
            "Positives, Reversed"
        );
        assertEquals(
            0.8,
            Util.rangePercent(-10, 0, -2),
            "Negatives, Normal"
        );
        assertEquals(
            0.2,
            Util.rangePercent(0, -10, -2),
            "Negatives, Reversed"
        );

    }
}
