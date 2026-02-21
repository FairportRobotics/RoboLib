package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.fairportrobotics.frc.robolib.trajectory.ballistics.exceptions.BallisticException;
import org.fairportrobotics.frc.robolib.trajectory.ballistics.models.BallisticModel;


public abstract class BallisticConstraintTest <T extends BallisticConstraint> {
    //
    //  Internal classes
    //

    public record TestConstraintEvalParams<U>(
        String test_name,
        BallisticModel<?> model,
        BCEvalParams evalParams,
        U constraint,
        Double result
    ) {}

    //
    //  Members
    //

    private double epsilon = 1e-5;

    //
    //  Methods
    //

    protected void testConstraintEval(List<TestConstraintEvalParams<T>> testParams) {
        for (TestConstraintEvalParams<T> params : testParams) {
            try {
                Double result = params.constraint.evaluate(params.model, params.evalParams);

                if(result == null || params.result == null) {
                    assertEquals(params.result, result, params.test_name);
                } else {
                    assertEquals(params.result, result, epsilon, params.test_name);
                }
            } catch(BallisticException e) {
                assertEquals("Got Ballistic Exception", e);
            }
        }
    }

}
