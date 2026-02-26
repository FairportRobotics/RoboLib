package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.ConstantBM;

import edu.wpi.first.units.Units;

import org.junit.jupiter.api.Test;

public class RobotLauncherVelBCTest extends BallisticConstraintTest<RobotLauncherVelBC> {

    private enum DegreeOfFreedom {
        SPEED,
        ELEVATION,
        AZIMUTH;

        public String prettyName() {
            switch(this) {
                case SPEED:
                    return "Speed";
                case ELEVATION:
                    return "Elevation";
                case AZIMUTH:
                    return "Azimuth";
                default:
                    return this.name();
            }
        }
    };

    private enum ParamCategory {
        LOW_OOB,
        LOW_FAR,
        LOW_MID,
        LOW_NEAR,
        HIGH_NEAR,
        HIGH_MID,
        HIGH_FAR,
        HIGH_OOB;

        public boolean isValid() {
            return this != LOW_OOB && this != HIGH_OOB;
        }

        public boolean isLow() {
            return  this == LOW_OOB ||
                    this == LOW_FAR ||
                    this == LOW_MID ||
                    this == LOW_NEAR;
        }

        public Double expectedValue() {
            switch (this) {
                case LOW_OOB:
                case HIGH_OOB:
                    return null;
                case LOW_NEAR:
                case HIGH_NEAR:
                    return 0.0;
                case LOW_MID:
                case HIGH_MID:
                    return 0.5;
                case LOW_FAR:
                case HIGH_FAR:
                    return 1.0;
                default:
                    return null;
            }
        }

        public String rangeString() {
            switch (this) {
                case LOW_OOB:
                case HIGH_OOB:
                    return "Out of Bounds";
                case LOW_NEAR:
                case HIGH_NEAR:
                    return "Near";
                case LOW_MID:
                case HIGH_MID:
                    return "Mid";
                case LOW_FAR:
                case HIGH_FAR:
                    return "Far";
                default:
                    return "Unknown (" + this.name() + ")";
            }
        }
    };

    //
    //  Param generator functions
    //

    private static TestConstraintEvalParams<RobotLauncherVelBC> mkParams(
        String name,
        Velocity3d candidateShooterRelativeVelocity,
        Double result
    ) {
        return mkParams(
            name,
            candidateShooterRelativeVelocity,
            new RobotLauncherVelBC(
                Units.MetersPerSecond.of(0.0), Units.MetersPerSecond.of(4.0), 0.5,
                Units.Degrees.of(-90), Units.Degrees.of(90), 0.5,
                Units.Degrees.of(0), Units.Degrees.of(80), 0.5
            ),
            result
        );
    }

    private static TestConstraintEvalParams<RobotLauncherVelBC> mkParams(
        String name,
        Velocity3d candidateShooterRelativeVelocity,
        RobotLauncherVelBC constraint,
        Double result
    ) {
        return new TestConstraintEvalParams<RobotLauncherVelBC>(
            name,
            new ConstantBM(),
            BCEvalParams.genSetShooterRelativeVelocity(candidateShooterRelativeVelocity),
            constraint,
            result
        );
    }

    private static List<TestConstraintEvalParams<RobotLauncherVelBC>> mkSingleDOFParams(DegreeOfFreedom dof) {
        // Get the maps of outputs
        Map<ParamCategory, Double> speedMap     = new HashMap<>();
        Map<ParamCategory, Double> elevationMap = new HashMap<>();
        Map<ParamCategory, Double> azimuthMap   = new HashMap<>();

        switch (dof) {
            case SPEED:
                speedMap = Map.of(
                    ParamCategory.LOW_OOB, -1.0,
                    ParamCategory.LOW_FAR, 0.0,
                    ParamCategory.LOW_MID, 0.5,
                    ParamCategory.LOW_NEAR, 1.0,
                    ParamCategory.HIGH_NEAR, 3.0,
                    ParamCategory.HIGH_MID, 3.5,
                    ParamCategory.HIGH_FAR, 4.0,
                    ParamCategory.HIGH_OOB, 5.0
                );
                break;
            case ELEVATION:
                elevationMap = Map.of(
                    ParamCategory.LOW_OOB, -10.0,
                    ParamCategory.LOW_FAR, 0.0,
                    ParamCategory.LOW_MID, 10.0,
                    ParamCategory.LOW_NEAR, 20.0,
                    ParamCategory.HIGH_NEAR, 60.0,
                    ParamCategory.HIGH_MID, 70.0,
                    ParamCategory.HIGH_FAR, 80.0,
                    ParamCategory.HIGH_OOB, 90.0
                );
            case AZIMUTH:
                azimuthMap = Map.of(
                    ParamCategory.LOW_OOB, -135.0,
                    ParamCategory.LOW_FAR, -90.0,
                    ParamCategory.LOW_MID, -67.5,
                    ParamCategory.LOW_NEAR, -45.0,
                    ParamCategory.HIGH_NEAR, 45.0,
                    ParamCategory.HIGH_MID, 67.5,
                    ParamCategory.HIGH_FAR, 90.0,
                    ParamCategory.HIGH_OOB, 135.0
                );
        }

        // Generate the test params
        List<TestConstraintEvalParams<RobotLauncherVelBC>> toRet = new LinkedList<>();
        for (ParamCategory category : ParamCategory.values()) {
            String testName =
                ((category.isValid()) ? "Pass" : "Fail") + " (" +
                ((category.isLow()) ? "Low" : "High") + " " + (dof.prettyName()) +
                ", " + category.rangeString() + ")";
            Velocity3d velocity = Velocity3d.genSpherical(
                speedMap.getOrDefault(category, 2.0),
                elevationMap.getOrDefault(category, 45.0),
                azimuthMap.getOrDefault(category, 0.0)
            );

            TestConstraintEvalParams<RobotLauncherVelBC>  testParams = 
                mkParams(testName, velocity, category.expectedValue());

            toRet.add(testParams);
        }
        return toRet;
    }

    //
    //  Tests
    //

    @Test void testIdeal() {
        this.testConstraintEval(List.of(
            mkParams(
                "Pass (Ideal)",
                Velocity3d.genSpherical(2.0, 45, 0),
                0.0
            )
        ));
    }

    @Test void testSpeed() {
        this.testConstraintEval(mkSingleDOFParams(DegreeOfFreedom.SPEED));
    }

    @Test void testElevation() {
        this.testConstraintEval(mkSingleDOFParams(DegreeOfFreedom.ELEVATION));
    }

    @Test void testAzimuth() {
        this.testConstraintEval(mkSingleDOFParams(DegreeOfFreedom.AZIMUTH));
    }

    @Test void testMultiFail() {
        this.testConstraintEval(List.of(
            mkParams(
                "Multi-Fail, Tie, Extreme",
                Velocity3d.genSpherical(4, 80, 0),
                1.0
            ),
            mkParams(
                "Multi-Fail, Tie, Mid",
                Velocity3d.genSpherical(3.5, 40, 0),
                0.5
            ),
            mkParams(
                "Multi-Fail, non-tie",
                Velocity3d.genSpherical(4, 40, 0),
                1.0
            )
        ));
    }
}
