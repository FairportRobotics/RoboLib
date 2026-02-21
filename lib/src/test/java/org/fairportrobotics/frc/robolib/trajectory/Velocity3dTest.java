package org.fairportrobotics.frc.robolib.trajectory;


import org.junit.jupiter.api.Test;

import edu.wpi.first.units.Units;

import static org.junit.jupiter.api.Assertions.*;

public class Velocity3dTest {
    private static final double epsilon=1e-10;

    //
    //  Test base create and coordinate space transform
    //

    void testConstructorBase(Velocity3d test) {
        // Check cartesian
        assertEquals(Math.cos(Math.toRadians(45)), test.getXVelocity().in(Units.MetersPerSecond), epsilon, "X");
        assertEquals(Math.sin(Math.toRadians(45)), test.getYVelocity().in(Units.MetersPerSecond), epsilon, "Y");
        assertEquals(1.0, test.getZVelocity().in(Units.MetersPerSecond), epsilon, "Z");

        // Check cylindrical
        assertEquals(1.0, test.getVeritcalVelocity().in(Units.MetersPerSecond), epsilon, "Vertical");
        assertEquals(1.0, test.getHorizontalVelocity().in(Units.MetersPerSecond), epsilon, "Horizontal");
        assertEquals(45, test.getAzimuthAngle().in(Units.Degrees), epsilon, "Azimuth");

        // Check spherical
        assertEquals(45, test.getElevationAngle().in(Units.Degrees), epsilon, "Elevation");
        assertEquals(Math.sqrt(2.0), test.getSpeed().in(Units.MetersPerSecond), epsilon, "Magnitude");

    }

    @Test void testCartesianConstructor() {
        Velocity3d test = new Velocity3d(
            Units.MetersPerSecond.of(Math.cos(Math.toRadians(45))),
            Units.MetersPerSecond.of(Math.sin(Math.toRadians(45))),
            Units.MetersPerSecond.of(1.0)
        );

        testConstructorBase(test);

        // Check implicit cast to LinearVelocity
        assertEquals(test, new Velocity3d(
                Math.cos(Math.toRadians(45)),
                Math.sin(Math.toRadians(45)),
                1.0
            ), "Implicit convert"
        );
    }

    @Test void testCylindicalConstructor() {
        testConstructorBase(new Velocity3d(
            Units.MetersPerSecond.of(1.0),
            Units.MetersPerSecond.of(1.0),
            Units.Degrees.of(45)
        ));
    }

    @Test void testSphericalConstructor() {
        Velocity3d test = new Velocity3d(
            Units.MetersPerSecond.of(Math.sqrt(2)),
            Units.Degrees.of(45),
            Units.Degrees.of(45)
        );
        testConstructorBase(test);
        assertEquals(
            test,
            Velocity3d.genSpherical(Math.sqrt(2), 45, 45),
            "genSpherical Test"
        );
    }

    //
    //  Test additional getters
    //

    @Test void testMagnitudes() {
        Velocity3d test = new Velocity3d(1.0, 1.0, 1.0);
        assertEquals(3.0, test.getSquareMagnitude(), epsilon, "Square magnitude");
        assertEquals(Math.sqrt(3.0), test.getMagnitude().in(Units.MetersPerSecond), epsilon, "Magnitude");
    }

    //
    //  Test transforms
    //

    @Test void testPlus() {
        assertEquals(
            new Velocity3d(5.0, 6.0, 7.0),
            new Velocity3d(4.0, 5.0, 6.0).plus(
                new Velocity3d(1.0, 1.0, 1.0)
            )
        );
    }

    @Test void testMinus() {
        Velocity3d test = new Velocity3d(1.0, 1.0, 1.0).minus(
            new Velocity3d(2.0, 2.0, 2.0)
        );

        assertEquals(new Velocity3d(-1.0, -1.0, -1.0), test, "Subtract");
    }

    @Test void testRotate() {
        Velocity3d test = new Velocity3d(
            Units.MetersPerSecond.of(1.0),
            Units.Degrees.of(45),
            Units.Degrees.of(45)
        ).rotate(Units.Degrees.of(45));

        assertEquals(
            new Velocity3d(
                Units.MetersPerSecond.of(1.0),
                Units.Degrees.of(45),
                Units.Degrees.of(90)
            ), test, "Rotate"
        );
    }
}
