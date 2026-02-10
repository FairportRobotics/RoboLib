package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.units.measure.Angle;

/**
 * Param block for ballistic constraint and calculator evaluation
 */
public class BCEvalParams {
    //
    //  Members
    //

    // Robot motion params
    private final Velocity3d currentGlobalRobotVelocity;
    private final Angle      currentGlobalRobotAngle;
    private final Velocity3d currentGlobalShooterVelocity;

    // Candidate params
    private final Velocity3d candidateGlobalTotalVelocity;

    // Composite value cache
    private Velocity3d cache_candidateGlobalShooterVelocity;
    private Velocity3d cache_candidateRelativeShooterVelocity;

    //
    // Setup
    //

    public BCEvalParams(
        Velocity3d currentGlobalRobotVelocity,
        Angle      currentGlobalRobotAngle,
        Velocity3d currentGlobalShooterVelocity,
        Velocity3d candidateGlobalTotalVelocity
    ) {
        this.currentGlobalRobotVelocity     = currentGlobalRobotVelocity;
        this.currentGlobalRobotAngle        = currentGlobalRobotAngle;
        this.currentGlobalShooterVelocity   = currentGlobalShooterVelocity;
        this.candidateGlobalTotalVelocity   = candidateGlobalTotalVelocity;

        this.cache_candidateGlobalShooterVelocity   = null;
        this.cache_candidateRelativeShooterVelocity = null;
    }


    //
    //  Base attribute getters
    //

    /**
     * @return The robot's current velocity (in field coordinates)
     */
    public Velocity3d getCurrentGlobalRobotVelocity() {
        return this.currentGlobalRobotVelocity;
    }


    /**
     * @return The robot's current heading angle (in field coordinates)
     */
    public Angle      getCurrentGlobalRobotAngle() {
        return this.currentGlobalRobotAngle;
    }


    /**
     * @return The shooter's current launch velocity (in field coordinates)
     */
    public Velocity3d getCurrentGlobalShooterVelocity() {
        return this.currentGlobalShooterVelocity;
    }

    /**
     * @return The candidate total (robot + shooter) velocity (in field coordinates)
     */
    public Velocity3d getCandidateGlobalTotalVelocity() {
        return this.candidateGlobalTotalVelocity;
    }

    //
    //  Composite value getters
    //

    /**
     * @return The candidate shooter velocity (in field coordinates) at current
     *  robot velocity
     */
    public Velocity3d getCandidateGlobalShooterVelocity() {
        if(this.cache_candidateGlobalShooterVelocity == null) {
            this.cache_candidateGlobalShooterVelocity =
                this.candidateGlobalTotalVelocity
                    .minus(this.currentGlobalRobotVelocity);
        }
        return this.cache_candidateGlobalShooterVelocity;
    }

    /**
     * @return The candidate shooter velocity (in robot coordinates) at current
     *  robot velocity and heading
     */
    public Velocity3d getCandidateRelativeShooterVelocity() {
        if(this.cache_candidateRelativeShooterVelocity == null) {
            this.cache_candidateRelativeShooterVelocity =
                new Velocity3d(
                    this.getCandidateGlobalShooterVelocity().getHorizontalVelocity(),
                    this.getCandidateGlobalShooterVelocity().getVeritcalVelocity(),
                    this.getCandidateGlobalShooterVelocity().getAzimuthAngle()
                        .minus(this.getCurrentGlobalRobotAngle())
                );
        }
        return this.cache_candidateRelativeShooterVelocity;
    }

}
