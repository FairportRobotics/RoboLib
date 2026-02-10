package org.fairportrobotics.frc.robolib.trajectory.ballistics.calculators;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

public class BCResult<P> {
    public final Double     velPenalty;     /// The penalty for the computed launcher velocity (null if invalid)
    public final Velocity3d velGlobal;      /// The computed launcher velocity (in the global coorinate space)
    public final Velocity3d velRelative;    /// The computed launcher velocity (in the robot-relative coordinate space)
    public final P          velParam;       /// The model parameter that yielded velGlobal


    public BCResult(
        Double      velPenalty,
        Velocity3d  velGlobal,
        Velocity3d  velRelative,
        P           velParam
    ) {
        this.velPenalty     = velPenalty;
        this.velGlobal      = velGlobal;
        this.velRelative    = velRelative;
        this.velParam       = velParam;
    }

    public boolean isValid() {
        return this.velPenalty != null;
    }
}
