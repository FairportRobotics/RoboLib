package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.units.Units;
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
    //  Setup
    //

    public static BCEvalParams genSetShooterVelocity(
        Velocity3d  candidateShooterVelocity
    ) {
        return genSetShooterVelocity(
            Velocity3d.zero, Units.Degrees.zero(), Velocity3d.zero,
            candidateShooterVelocity
        );
    }

    public static BCEvalParams genSetShooterVelocity(
        Velocity3d  robotVelocity,
        Angle       robotAngle,
        Velocity3d  currentShooterVelocity,
        Velocity3d  candidateShooterVelocity
    ) {
        return new BCEvalParams(
            robotVelocity,
            robotAngle,
            currentShooterVelocity,
            candidateShooterVelocity.plus(robotVelocity)
        );

    }

    public static BCEvalParams genSetShooterRelativeVelocity(
        Velocity3d candidateShooterVelocityRelative
    ) {
        return genSetShooterRelativeVelocity(
            Velocity3d.zero, Units.Degrees.zero(), Velocity3d.zero,
            candidateShooterVelocityRelative
        );
    }

    public static BCEvalParams genSetShooterRelativeVelocity(
        Velocity3d robotVelocity,
        Angle robotAngle,
        Velocity3d currentShooterVelocity,
        Velocity3d candidateShooterVelocityRelative
    ) {
        BCEvalParams toRet = genSetShooterVelocity(
            robotVelocity,
            robotAngle,
            currentShooterVelocity,
            candidateShooterVelocityRelative.rotate(robotAngle)
        );
        return toRet;
    }

    public BCEvalParams(
        Velocity3d robotVelocity,
        Angle      robotAngle,
        Velocity3d currentShooterVelocity,
        Velocity3d candidateVelocity
    ) {
        this.robotVelocity                          = robotVelocity;
        this.robotAngle                             = robotAngle;
        this.currentShooterVelocity                 = currentShooterVelocity;

        this.candidateVelocity                      = candidateVelocity;

        this.cache_candidateShooterVelocity         = null;
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

    public int hashCode() {
        return  this.robotVelocity.hashCode() ^
                Double.hashCode(this.robotAngle.in(Units.Degrees)) ^
                this.currentShooterVelocity.hashCode() ^
                this.candidateVelocity.hashCode();
    }

    public String toString() {
        return String.format(
            "BCEvalParams(robotVelocity: %s, robotAngle: %s, currentShooterVelocity: %s, candidateVelocity: %s)",
            this.robotVelocity.toString(),
            this.robotAngle.toString(),
            this.currentShooterVelocity.toString(),
            this.candidateVelocity.toString()
        );
    }

    public boolean equals(BCEvalParams other) {
        return
            this.robotVelocity.equals(other.getRobotVelocity()) &&
            this.robotAngle.equals(other.getRobotAngle()) &&
            this.currentShooterVelocity.equals(other.getCurrentShooterVelocity()) &&
            this.candidateVelocity.equals(other.candidateVelocity);
    }

    public boolean equals(Object other) {
        return false;
    }

}
