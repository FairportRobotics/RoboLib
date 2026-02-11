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
    private final Velocity3d robotVelocity;
    private final Angle      robotAngle;
    private final Velocity3d currentShooterVelocity;

    // Candidate params
    private final Velocity3d candidateVelocity;

    // Composite value cache
    private Velocity3d cache_candidateShooterVelocity;
    private Velocity3d cache_candidateShooterVelocityRelative;

    //
    // Setup
    //

    public BCEvalParams(
        Velocity3d robotVelocity,
        Angle      robotAngle,
        Velocity3d currentShooterVelocity,
        Velocity3d candidateVelocity
    ) {
        this.robotVelocity          = robotVelocity;
        this.robotAngle             = robotAngle;
        this.currentShooterVelocity = currentShooterVelocity;

        this.candidateVelocity      = candidateVelocity;

        this.cache_candidateShooterVelocity   = null;
        this.cache_candidateShooterVelocityRelative = null;
    }


    //
    //  Base attribute getters
    //

    /**
     * @return The robot's current velocity (in field coordinates)
     */
    public Velocity3d getRobotVelocity() {
        return this.robotVelocity;
    }


    /**
     * @return The robot's current heading angle (in field coordinates)
     */
    public Angle      getRobotAngle() {
        return this.robotAngle;
    }


    /**
     * @return The shooter's current launch velocity (in field coordinates)
     */
    public Velocity3d getCurrentShooterVelocity() {
        return this.currentShooterVelocity;
    }


    /**
     * @return The candidate total (robot + shooter) velocity (in field coordinates)
     */
    public Velocity3d getCandidateVelocity() {
        return this.candidateVelocity;
    }

    //
    //  Composite value getters
    //

    /**
     * @return The candidate shooter velocity (in field coordinates) at current
     *  robot velocity
     */
    public Velocity3d getCandidateShooterVelocity() {
        if(this.cache_candidateShooterVelocity == null) {
            this.cache_candidateShooterVelocity =
                this.candidateVelocity
                    .minus(this.robotVelocity);
        }
        return this.cache_candidateShooterVelocity;
    }

    /**
     * @return The candidate shooter velocity (in robot coordinates) at current
     *  robot velocity and heading
     */
    public Velocity3d getCandidateShooterVelocityRelative() {
        if(this.cache_candidateShooterVelocityRelative == null) {
            this.cache_candidateShooterVelocityRelative =
                this.getCandidateShooterVelocity().rotate(this.robotAngle.times(-1));
        }
        return this.cache_candidateShooterVelocityRelative;
    }

}
