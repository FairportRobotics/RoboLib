package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import org.junit.jupiter.api.Test;

import edu.wpi.first.units.Units;

import static org.junit.jupiter.api.Assertions.*;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

public class BCEvalParamsTest {
    @Test void testCandidateShooterVelocity() {
        assertEquals(
            new Velocity3d(1.0, 2.0, 3.0),
            new BCEvalParams(
                new Velocity3d(1.0, 1.0, 1.0),
                Units.Degrees.of(0),
                Velocity3d.zero,
                new Velocity3d(2.0, 3.0, 4.0)
            ).getCandidateShooterVelocity(),
            "Robot Velocity Only"
        );

        assertEquals(
            new Velocity3d(1.0, 0.0, 0.0),
            new BCEvalParams(
                Velocity3d.zero,
                Units.Degrees.of(90),
                Velocity3d.zero,
                new Velocity3d(0.0, 1.0, 0.0)
            ).getCandidateShooterVelocityRelative(),
            "Robot Angle Only"
        );

        assertEquals(
            new Velocity3d(1.0, 0.0, 0.0),
            new BCEvalParams(
                new Velocity3d(1.0, 1.0, 0.0),
                Units.Degrees.of(90),
                Velocity3d.zero,
                new Velocity3d(1.0, 2.0, 0.0)
            ).getCandidateShooterVelocityRelative(),
            "Robot Velocity and Angle"
        );
    }

    @Test void testGenSetShooterVelocity() {
        Velocity3d shooterVelocity = Velocity3d.genSpherical(2.0, -135.0, 0.0);
        assertEquals(
            shooterVelocity,
            BCEvalParams.genSetShooterVelocity(shooterVelocity).getCandidateShooterVelocity()
        );
    }

    @Test void testGenSetShooterRelativeVelocity() {
        Velocity3d shooterRelativeVelocity = Velocity3d.genSpherical(2.0, -135.0, 0.0);
        assertEquals(
            shooterRelativeVelocity,
            BCEvalParams.genSetShooterRelativeVelocity(shooterRelativeVelocity).getCandidateShooterVelocityRelative()
        );
    }
}
