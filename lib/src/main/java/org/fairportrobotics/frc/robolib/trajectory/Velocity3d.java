package org.fairportrobotics.frc.robolib.trajectory;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class Velocity3d {
    private final LinearVelocity vX;
    private final LinearVelocity vY;
    private final LinearVelocity vZ;

    private Double          cache_squareMagnitudeVelocity;
    private LinearVelocity  cache_magnitudeVelocity;
    private Angle           cache_eleveationAngle;
    private Angle           cache_azimuthAngle;
    private LinearVelocity  cache_horizontalVelocity;

    public Velocity3d(LinearVelocity vX, LinearVelocity vY, LinearVelocity vZ) {
        this.vX = vX.copy();
        this.vY = vY.copy();
        this.vZ = vZ.copy();

        this.cache_squareMagnitudeVelocity  = null;
        this.cache_magnitudeVelocity        = null;
        this.cache_eleveationAngle          = null;
        this.cache_azimuthAngle             = null;
        this.cache_horizontalVelocity       = null;
    }


    public Velocity3d(double vX, double vY, double vZ) {
        this(
            Units.MetersPerSecond.of(vX),
            Units.MetersPerSecond.of(vY),
            Units.MetersPerSecond.of(vZ)
        );
    }


    public Velocity3d(LinearVelocity horizontalVelocity, LinearVelocity verticalVelocity, Angle azimuthAngle) {
        this(
            horizontalVelocity.times(Math.cos(azimuthAngle.in(Units.Radians))),
            horizontalVelocity.times(Math.sin(azimuthAngle.in(Units.Radians))),
            verticalVelocity
        );
        this.cache_azimuthAngle         = azimuthAngle.copy();
        this.cache_horizontalVelocity   = horizontalVelocity.copy();
    }


    public Velocity3d(LinearVelocity radialVelocity, Angle elevationAngle, Angle azimuthAngle) {
        this(
            radialVelocity.times(Math.cos(elevationAngle.in(Units.Radians))),
            radialVelocity.times(Math.sin(elevationAngle.in(Units.Radians))),
            azimuthAngle
        );
        this.cache_eleveationAngle      = elevationAngle.copy();
        this.cache_magnitudeVelocity    = radialVelocity.copy();
    }


    //
    //  Cartesian Coordinate Getters
    //

    /**
     * @return  The x-axis component of the velocity
     */
    public LinearVelocity getXVelocity() {
        return vX;
    }


    /**
     * @return  The y-axis component of the velocity
     */
    public LinearVelocity getYVelocity() {
        return vY;
    }


    /**
     * @return  The z-axis component of the velocity
     */
    public LinearVelocity getZVelocity() {
        return vZ;
    }


    /**
     * @return The squared magnitude of the velocity vector in m^2s^-2
     */
    public double getSquareMagnitude() {
        if(this.cache_squareMagnitudeVelocity == null) {
            double vX_mps = this.vX.in(Units.MetersPerSecond);
            double vY_mps = this.vY.in(Units.MetersPerSecond);
            double vZ_mps = this.vZ.in(Units.MetersPerSecond);
            this.cache_squareMagnitudeVelocity =    Math.pow(vX_mps, 2) +
                                                    Math.pow(vY_mps, 2) +
                                                    Math.pow(vZ_mps, 2);
        }
        return this.cache_squareMagnitudeVelocity;
    }

    /**
     * @return  The magnitude of the velocity vector
     */
    public LinearVelocity getMagnitude() {
        if(this.cache_magnitudeVelocity == null) {
            this.cache_magnitudeVelocity = Units.MetersPerSecond.of(Math.pow(
                this.getSquareMagnitude(), 0.5
            ));
        }
        return this.cache_magnitudeVelocity;
    }


    //
    //  Cylindrical Coordinate getters
    //

    /**
     * @return  The magnitude of the vertical component of the velocity (alias of this.getVZ())
     */
    public LinearVelocity getVeritcalVelocity() {
        return this.getZVelocity();
    }


    /**
     * @return  The magnitude of the horizontal component of the velocity
     */
    public LinearVelocity getHorizontalVelocity() {
        if (this.cache_horizontalVelocity == null) {
            double vX_mps = this.getXVelocity().in(Units.MetersPerSecond);
            double vY_mps = this.getYVelocity().in(Units.MetersPerSecond);
            this.cache_horizontalVelocity = Units.MetersPerSecond.of(
                Math.pow(
                    Math.pow(vX_mps, 2) + Math.pow(vY_mps, 2),
                    0.5
                )
            );
        }
        return this.cache_horizontalVelocity;
    }


    /**
     * Returns the azimuth angle of the robot with respect to +X (CCW positive)
     * 
     * Azimuth is the angle in the horizontal (XY) plane, around the vertical (Z) axis.
     * 
     * @return The azimuth angle of the velocity, with respect to your chosen X
     */
    public Angle getAzimuthAngle() {
        if (this.cache_azimuthAngle == null) {
            double vX_mps = this.getXVelocity().in(Units.MetersPerSecond);
            double vY_mps = this.getYVelocity().in(Units.MetersPerSecond);
            this.cache_azimuthAngle = Units.Radians.of(Math.atan2(vY_mps, vX_mps));
        }
        return this.cache_azimuthAngle;
    }


    //
    //  Spherical Coordinate getters
    //


    /**
     * Returns the elevation angle of the robot with resepect to the horizontal plane
     * 
     * Elevation is the angle between the horizontal component of the velocity and the final
     * velocity. By convention, upwards angles are positive
     *
     * @return The elevation angle of the velocity, with respect to the horizontal (XY) components
     * of the velocity.
     */
    public Angle getElevation() {
        if(this.cache_eleveationAngle == null) {
            double vZ_mps       = this.getZVelocity().in(Units.MetersPerSecond);
            double vHori_mps    = this.getHorizontalVelocity().in(Units.MetersPerSecond);
            this.cache_eleveationAngle = Units.Radians.of(Math.atan2(vZ_mps, vHori_mps));
        }
        return this.cache_eleveationAngle;
    }


    /**
     * @return The radial component of the spherical velocity (alias of this.getMagnitude())
     */
    public LinearVelocity getSphericalVelocity() {
        return this.getMagnitude();
    }


    //
    // Transforms and Conversions
    //

    /**
     * Subtract other from this vector and return the resulting vector
     * @param other The vector to subtract from this one
     * @return The difference of this vector from other
     */
    public Velocity3d minus(Velocity3d other) {
        return new Velocity3d(
            this.vX.minus(other.vX),
            this.vY.minus(other.vY),
            this.vZ.minus(other.vZ)
        );
    }

    /**
     * Rotate the vector by adding a given angle to the azimuth
     * @param rotAngle The angle to add to the azimuth
     * @return A new, rotated Velocity3d instance
     */
    public Velocity3d rotate(Angle rotAngle) {
        return new Velocity3d(
            this.getHorizontalVelocity(),
            this.getVeritcalVelocity(),
            this.getAzimuthAngle().plus(rotAngle)
        );
    }


    /**
     * Multiply the velocity by time in order to get a displacement
     * @param time The time to compute over
     * @return The resulting displacement
     */
    public Translation3d times(Time time) {
        return new Translation3d(
            this.vX.times(time),
            this.vY.times(time),
            this.vZ.times(time)
        );
    }


    //
    //  Overrides
    //

    @Override
    public int hashCode() {
        return this.vX.hashCode() ^ this.vY.hashCode() ^ this.vZ.hashCode();
    }


    @Override
    public String toString() {
        return String.format("Velocity3d(X: %s, Y: %s, Z: %s)", vX, vY, vZ);
    }


    @Override
    public boolean equals(Object obj) {
        if(Velocity3d.class.isInstance(obj)) {
            Velocity3d otherVelocity = (Velocity3d) obj;
            return
                this.vX == otherVelocity.getXVelocity() &&
                this.vY == otherVelocity.getYVelocity() &&
                this.vZ == otherVelocity.getZVelocity();
        }
        return false;
    }
}
