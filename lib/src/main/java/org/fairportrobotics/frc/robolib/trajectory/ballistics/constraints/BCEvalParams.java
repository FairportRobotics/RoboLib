package org.fairportrobotics.frc.robolib.trajectory.ballistics.constraints;

import org.fairportrobotics.frc.robolib.trajectory.Velocity3d;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * Parameter block for Ballistic Constraint and Ballistic Caluclator evaluation
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

    /**
     * Generate a BCEvalParams instance for a zero velocity, zero angle
     * robot and shooter with a given shooter velocity.
     *
     * @param candidateShooterVelocity The targeted shooter velocity
     * @return The generated BCEvalParams instance
     */
    public static BCEvalParams genSetShooterVelocity(
        Velocity3d  candidateShooterVelocity
    ) {
        return genSetShooterVelocity(
            Velocity3d.kZero, Units.Degrees.zero(), Velocity3d.kZero,
            candidateShooterVelocity
        );
    }

    /**
     * Generate a robotVelocity instance with a given shooter velocity
     *
     * @param robotVelocity The robot's velocity
     * @param robotAngle The current robot angle
     * @param currentShooterVelocity The current velocity imparted by the shooter
     * @param candidateShooterVelocity The ideal velocity imparted by the shooter
     * @return The generated BCEvalParams instance
     */
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

    /**
     * Generate an instance with a given shooter relative velocity (from a 
     * zero-velocity, zero-angle shooter and robot).
     *
     * @param candidateShooterVelocityRelative The targeted relative shooter
     * velocity
     * @return The generated BCEvalParams instance
     */
    public static BCEvalParams genSetShooterRelativeVelocity(
        Velocity3d candidateShooterVelocityRelative
    ) {
        return genSetShooterRelativeVelocity(
            Velocity3d.kZero, Units.Degrees.zero(), Velocity3d.kZero,
            candidateShooterVelocityRelative
        );
    }

    /**
     * Generate an instance with a given shooter relative velocity (from a 
     * zero-velocity, zero-angle shooter and robot).
     *
     * @param robotVelocity The robot velocity
     * @param robotAngle The robot angle
     * @param currentShooterVelocity The current shooter velocity
     * @param candidateShooterVelocityRelative The targeted relative shooter
     * velocity
     * @return The generated BCEvalParams instance
     */
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

    /**
     * Main constructor (all params in field-relative coordinates)
     *
     * @param robotVelocity The robot's current velocity
     * @param robotAngle The robot's current angle (counter clockwise from +X)
     * @param currentShooterVelocity The current velocity imparted by the shooter
     * @param candidateVelocity The candidate total (robot + shooter) velocity
     * to evaluate
     */
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
     * Get the robot's current velocity (in field-relative coordinates)
     *
     * @return The robot's current velocity
     */
    public Velocity3d getRobotVelocity() {
        return this.robotVelocity;
    }


    /**
     * Get the robot's current heading angle (in field-relative coordinates)
     *
     * @return The robot's current heading angle
     */
    public Angle      getRobotAngle() {
        return this.robotAngle;
    }


    /**
     * Get the shooter's current launch velocity (in field-relative coordinates).
     *
     * @return The shooter's current launch velocity
     */
    public Velocity3d getCurrentShooterVelocity() {
        return this.currentShooterVelocity;
    }


    /**
     * Get the total (robot + shooter) candidate velocity (in field-relative coordinates).
     * 
     * @return The total candidate velocity
     */
    public Velocity3d getCandidateVelocity() {
        return this.candidateVelocity;
    }

    //
    //  Composite value getters
    //

    /**
     * Get the candidate shooter velocity (in field-relative coordinates) at the
     * current robot velocity.
     *
     * @return The candidate shooter velocity
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
     * Get the candidate shooter velocity (in robot-relative coordinates) at the
     * current robot velocity and heading
     *
     * @return The candidate shooter velocity (in robot-relative coordinates)
     */
    public Velocity3d getCandidateShooterVelocityRelative() {
        if(this.cache_candidateShooterVelocityRelative == null) {
            this.cache_candidateShooterVelocityRelative =
                this.getCandidateShooterVelocity().rotate(this.robotAngle.times(-1));
        }
        return this.cache_candidateShooterVelocityRelative;
    }

    @Override
    public int hashCode() {
        return  this.robotVelocity.hashCode() ^
                Double.hashCode(this.robotAngle.in(Units.Degrees)) ^
                this.currentShooterVelocity.hashCode() ^
                this.candidateVelocity.hashCode();
    }

    @Override
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

    @Override
    public boolean equals(Object other) {
        if(other instanceof BCEvalParams) {
            return this.equals((BCEvalParams) other);
        }
        return false;
    }

}
