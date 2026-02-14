package org.fairportrobotics.frc.robolib.trajectory.ballistics.models;


import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;

import static org.junit.jupiter.api.Assertions.*;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

public class ParabolicTimeOfFlightBMTest {
    private ParabolicTimeOfFlightBM getModel() {
        return new ParabolicTimeOfFlightBM(
            new Translation3d(Units.Meters.of(1), Units.Meters.of(1), Units.Meters.of(1)),
            new Translation3d(Units.Meters.of(4), Units.Meters.of(5), Units.Meters.of(2)),
            Units.Seconds.of(1.0), Units.Seconds.of(3), Units.Seconds.of(1.0)
        );
    }

    @Test void testParamIterator() {
        ParabolicTimeOfFlightBM model = getModel();

        // Define the initial param
        Time initParam = model.getInitParam();
        assertEquals(2.0, initParam.in(Units.Seconds), "Init param");
        assertArrayEquals(
            new Time[]{Units.Seconds.of(1.0), Units.Seconds.of(3.0)},
            model.getNeighborParams(initParam), "Neighbors"
        );
    }


    @Test void testSolutionVelocity() {
        ParabolicTimeOfFlightBM model = getModel();
        Time tof = Units.Seconds.of(1.0);

        // Compute the solution
        Velocity3d result = model.getCandidateVelocity(tof);
        assertEquals(new Velocity3d(
            Units.MetersPerSecond.of(5),
            Units.MetersPerSecond.of(1).minus(
                ParabolicTimeOfFlightBM.k_gravity.times(0.5).times(tof)
            ),
            Units.Radians.of(Math.atan2(4, 3))
        ), result, "Compute solution");

        // Check that positions match expected values
        assertEquals(
            model.getPosLaunch(),
            model.positionAtTime(result, Units.Seconds.of(0)),
            "Launch position"
        );
        assertEquals(
            model.getPosTarget(),
            model.positionAtTime(result, tof),
            "Target position check"
        );

        Time midTime = Units.Seconds.of(0.5);
        assertEquals(
            new Translation3d(
                Units.Meters.of(2.5),   // 1+3*0.5
                Units.Meters.of(3),     // 1+4*0.5
                Units.Meters.of(1.0)
                    .plus(result.getZVelocity().times(midTime))
                    .plus(
                        ParabolicTimeOfFlightBM.k_gravity
                            .times(0.5).times(midTime).times(midTime)
                    )
            ),
            model.positionAtTime(result, midTime),
            "Mid-flight position check"
        );
    }

    @Test void testReverseFunctions() {
        ParabolicTimeOfFlightBM model   = getModel();
        Time tof                        = Units.Seconds.of(1.0);
        Velocity3d solution             = model.getCandidateVelocity(tof);

        // Test initial position
        Time t0 = Units.Seconds.of(0.0);
        assertEquals(t0, model.getTimeAtRadius(solution, Units.Meters.of(0.0)), "Init Position (Rad)");
        assertEquals(t0, model.getTimeAtX(solution, model.getPosLaunch().getMeasureX()), "Init Position (X, abs)");
        assertEquals(t0, model.getTimeAtY(solution, model.getPosLaunch().getMeasureY()), "Init Position (Y, abs)");
        assertEquals(t0, model.getTimeAtXRelative(solution, Units.Meters.of(0.0)), "Init Position (X, rel)");
        assertEquals(t0, model.getTimeAtYRelative(solution, Units.Meters.of(0.0)), "Init Position (Y, rel)");

        // Test target position
        assertEquals(tof, model.getTimeAtRadius(solution, model.getRelativeHorizontalDistance()), "Final Position (Rad)");
        assertEquals(tof, model.getTimeAtX(solution, model.getPosTarget().getMeasureX()), "Final Position (X, Abs)");
        assertEquals(tof, model.getTimeAtY(solution, model.getPosTarget().getMeasureY()), "Final Position (Y, Abs)");
        assertEquals(tof, model.getTimeAtXRelative(solution, model.getTargetPosRelative().getMeasureX()), "Final Position (X, Rel)");
        assertEquals(tof, model.getTimeAtYRelative(solution, model.getTargetPosRelative().getMeasureY()), "Final Position (Y, Rel)");
        assertEquals(tof, model.getTimeAtTarget(solution));

        // Test mid-flight position
        Time tMid = Units.Seconds.of(0.5);
        assertEquals(tMid, model.getTimeAtRadius(solution, model.getRelativeHorizontalDistance().div(2)), "Mid Position (Rad)");
        assertEquals(tMid, model.getTimeAtX(solution, Units.Meters.of(2.5)), "Mid Position (X, Abs)");
        assertEquals(tMid, model.getTimeAtY(solution, Units.Meters.of(3.0)), "Mid Position (Y, Abs)");
        assertEquals(tMid, model.getTimeAtXRelative(solution, Units.Meters.of(1.5)), "Mid Position (X, REL)");
        assertEquals(tMid, model.getTimeAtYRelative(solution, Units.Meters.of(2.0)), "Mid Position (Y, REL)");
    }

    @Test void testCandidateTracking() {
        ParabolicTimeOfFlightBM model = getModel();
        Time[] candidates = {
            Units.Seconds.of(1.0),
            Units.Seconds.of(1.5),
            Units.Seconds.of(2.0),
            Units.Seconds.of(2.5),
            Units.Seconds.of(3.0)
        };

        for(int cur_candidate = 0; cur_candidate < candidates.length; ++cur_candidate) {
            // Check that existing candidates have been checked
            for(int seen_candidate = 0; seen_candidate < cur_candidate; ++seen_candidate) {
                assertTrue(
                    model.paramComputed(candidates[seen_candidate]),
                    String.format(
                        "Cur Candidate: %d, Seen Candidate: %d",
                        cur_candidate, seen_candidate
                    )
                );
            }

            // Check that unseen candidates haven't been checked
            for(int unseen_candidate = cur_candidate; unseen_candidate < candidates.length; ++unseen_candidate) {
                assertFalse(
                    model.paramComputed(candidates[unseen_candidate]),
                    String.format(
                        "Cur Candidate: %d, Unseen Candidate: %d",
                        cur_candidate, unseen_candidate
                    )
                );
            }

            // Compute for the candidate
            model.getCandidateVelocity(candidates[cur_candidate]);
        }
    }
}
