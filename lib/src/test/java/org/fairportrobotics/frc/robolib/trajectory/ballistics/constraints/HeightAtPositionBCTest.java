package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import java.util.List;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints.HeightAtPositionBC.PositionType;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.ConstantBM;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import org.junit.jupiter.api.Test;

public class HeightAtPositionBCTest extends BallisticConstraintTest<HeightAtPositionBC> {

    //
    //  Tests
    //

    private TestConstraintEvalParams<HeightAtPositionBC> mkParams(
        String name, Distance resultHeight, PositionType positionType,
        Double expected
    ) {
        return this.mkParams(
            name, resultHeight, Units.Meters.of(0.5), positionType,
            expected
        );
    }

    private TestConstraintEvalParams<HeightAtPositionBC> mkParams(
        String name, Distance resultHeight, Distance testPos,
        PositionType positionType, Double expected
    ) {
        return this.mkParams(
            name,
            resultHeight,
            Units.Meters.of(0.0),
            Units.Meters.of(1.0),
            testPos,
            positionType,
            expected
        );
    }

    private TestConstraintEvalParams<HeightAtPositionBC> mkParams(
        String name, Distance resultHeight, Distance minHeight,
        Distance safeHeight, Distance testPos, PositionType positionType,
        Double expected
    ) {
        return this.mkParams(
            name,
            new ConstantBM(
                Translation3d.kZero,
                new Translation3d(
                    Units.Meters.of(1.0),
                    Units.Meters.of(1.0),
                    Units.Meters.of(1.0)
                )
            ).setPositionResult(
                Units.Meters.of(0.0),
                Units.Meters.of(0.0),
                resultHeight
            ),
            minHeight,
            safeHeight,
            testPos,
            positionType,
            expected
        );
    }

    private TestConstraintEvalParams<HeightAtPositionBC> mkParams(
        String name, ConstantBM model, Distance minHeight, Distance safeHeight,
        Distance testPos, PositionType positionType, Double expected
    ) {
        return new TestConstraintEvalParams<HeightAtPositionBC>(
            name,
            model,
            new BCEvalParams(
                Velocity3d.kZero, Units.Degrees.of(0), Velocity3d.kZero, Velocity3d.kZero
            ),
            new HeightAtPositionBC(
                minHeight,
                safeHeight,
                testPos,
                positionType
            ),
            expected
        );
    }

    @Test void testHeightEval() {
        for (PositionType positionType : PositionType.values()) {
            this.testConstraintEval(List.of(
                this.mkParams(
                    "Pass (Ideal, " + positionType.name() + ')',
                    Units.Meters.of(2.0),
                    PositionType.TargetRadius,
                    0.0
                ),
                this.mkParams(
                    "Fail (" + positionType.name() + ')',
                    Units.Meters.of(-1.0),
                    PositionType.TargetRadius,
                    null
                ),
                this.mkParams(
                    "Pass (Near Border, " + positionType.name() + ')',
                    Units.Meters.of(    1.0),
                    PositionType.TargetRadius,
                    1.0
                ),
                this.mkParams(
                    "Pass (Mid, " + positionType.name() + ')',
                    Units.Meters.of(0.5),
                    PositionType.TargetRadius,
                    0.5
                ),
                this.mkParams(
                    "Pass (Far Edge, " + positionType.name() + ')',
                    Units.Meters.of(0.0),
                    PositionType.TargetRadius,
                    0.0
                )
            ));
        }
    }

}
