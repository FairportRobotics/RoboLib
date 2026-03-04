package org.fairportrobotics.frc.robolib.trajectory;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * <p>A class defining a 3-dimensional velocity vector. Vector can be specified
 * and queried in cartesian, cylindrical, and spherical coordinates, and may be
 * transformed.</p>
 * 
 * <p>Velocity3d instances are immutable (i.e. they cannot be changed), are
 * hashable (allowing them to be used in HashMap and HashSet), and provide
 * equality checks.</p>
 */
public class Velocity3d {
    /** A Velocity3d instance with 0 velocity in all directions */
    public static final Velocity3d kZero = new Velocity3d(0.0, 0.0, 0.0);

    private final LinearVelocity xVelocity;
    private final LinearVelocity yVelocity;
    private final LinearVelocity zVelocity;

    private Double          cache_squareMagnitudeVelocity;
    private LinearVelocity  cache_magnitudeVelocity;
    private Angle           cache_eleveationAngle;
    private Angle           cache_azimuthAngle;
    private LinearVelocity  cache_horizontalVelocity;

    /**
     * Construct a velocity vector with cartesian coordinates
     *
     * @param xVelocity The velocity in the X dimension
     * @param yVelocity The velocity in the Y dimension
     * @param zVelocity The velocity in the vertical/Z dimension
     */
    public Velocity3d(LinearVelocity xVelocity, LinearVelocity yVelocity, LinearVelocity zVelocity) {
        this.xVelocity = xVelocity.copy();
        this.yVelocity = yVelocity.copy();
        this.zVelocity = zVelocity.copy();

        this.cache_squareMagnitudeVelocity  = null;
        this.cache_magnitudeVelocity        = null;
        this.cache_eleveationAngle          = null;
        this.cache_azimuthAngle             = null;
        this.cache_horizontalVelocity       = null;
    }


    /**
     * Construct a velocity vector with cartesian coordinates
     *
     * @param xVelocityMPS The velocity in the X dimension (in meters per second)
     * @param yVelocityMPS The velocity in the Y dimension (in meters per second)
     * @param zVelocityMPS The velocity in the vertical/Z dimension (in meters per second)
     */
    public Velocity3d(double xVelocityMPS, double yVelocityMPS, double zVelocityMPS) {
        this(
            Units.MetersPerSecond.of(xVelocityMPS),
            Units.MetersPerSecond.of(yVelocityMPS),
            Units.MetersPerSecond.of(zVelocityMPS)
        );
    }


    /**
     * Construct a velocity in cylindrical coordinates (around the vertical axis).
     *
     * @param horizontalVelocity The velocity parallel to the ground
     * @param verticalVelocity The vertical velocity
     * @param azimuthAngle The angle of the velocity around the vertical axis,
     * counterclockwise from +X
     */
    public Velocity3d(LinearVelocity horizontalVelocity, LinearVelocity verticalVelocity, Angle azimuthAngle) {
        this(
            horizontalVelocity.times(Math.cos(azimuthAngle.in(Units.Radians))),
            horizontalVelocity.times(Math.sin(azimuthAngle.in(Units.Radians))),
            verticalVelocity
        );
    }

    /**
     * Construct a velocity vector in spherical coordinates
     *
     * @param speedMPS The magnitude of the vector (in meters per second)
     * @param elevationAngleDeg The elevation angle of the vector (in degrees,
     * angle above the horizontal plane)
     * @param azimuthAngleDeg The angle of the vector around the vertical
     * axis (in degrees), counterclockwise from +X
     * @return The generated velocity vector
     */
    public static Velocity3d genSpherical(double speedMPS, double elevationAngleDeg, double azimuthAngleDeg) {
        return new Velocity3d(
            Units.MetersPerSecond.of(speedMPS),
            Units.Degrees.of(elevationAngleDeg),
            Units.Degrees.of(azimuthAngleDeg)
        );
    }

    /**
     * Construct a velocity vector in spherical coordinates
     *
     * @param speed The magnitude of the vector
     * @param elevationAngle The elevation angle of the vector (angle above the
     * horizontal plane)
     * @param azimuthAngle The angle of the vector around the vertical axis,
     * counterclockwise from +X
     */
    public Velocity3d(LinearVelocity speed, Angle elevationAngle, Angle azimuthAngle) {
        this(
            speed.times(Math.cos(elevationAngle.in(Units.Radians))),
            speed.times(Math.sin(elevationAngle.in(Units.Radians))),
            azimuthAngle
        );
    }


    //
    //  Cartesian Coordinate Getters
    //

    /**
     * Get the x-axis component of the velocity
     *
     * @return  The x-axis component of the velocity
     */
    public LinearVelocity getXVelocity() {
        return xVelocity;
    }


    /**
     * Get the y-axis component of the velocity
     *
     * @return  The y-axis component of the velocity
     */
    public LinearVelocity getYVelocity() {
        return yVelocity;
    }


    /**
     * Get the z-axis component of the velocity
     *
     * @return  The z-axis component of the velocity
     */
    public LinearVelocity getZVelocity() {
        return zVelocity;
    }


    /**
     * Returns the squared magnitude of the velocity vector in m^2s^-2
     *
     * @return The squared magnitude of the velocity vector in m^2s^-2
     */
    public double getSquareMagnitude() {
        if(this.cache_squareMagnitudeVelocity == null) {
            double vX_mps = this.xVelocity.in(Units.MetersPerSecond);
            double vY_mps = this.yVelocity.in(Units.MetersPerSecond);
            double vZ_mps = this.zVelocity.in(Units.MetersPerSecond);
            this.cache_squareMagnitudeVelocity =    Math.pow(vX_mps, 2) +
                                                    Math.pow(vY_mps, 2) +
                                                    Math.pow(vZ_mps, 2);
        }
        return this.cache_squareMagnitudeVelocity;
    }

    /**
     * Returns the magnitude/speed of the velocity vector
     *
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

    /**
     * Alias of this.getMagnitude()
     *
     * @return The magnitude of the velocity vector
     */
    public LinearVelocity getSpeed() {
        return this.getMagnitude();
    }


    //
    //  Cylindrical Coordinate getters
    //

    /**
     * Returns the vertical component of the velocity
     *
     * @return  The magnitude of the vertical component of the velocity (alias of this.getVZ())
     */
    public LinearVelocity getVeritcalVelocity() {
        return this.getZVelocity();
    }


    /**
     * Returns the horizontal component of the velocity
     *
     * @return  The the horizontal component of the velocity
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
     * <p>Returns the azimuth angle of the robot (counterclockwise from +X).</p>
     * 
     * <p>Azimuth is the angle in the horizontal (XY) plane around the vertical
     * (Z) axis.</p>
     * 
     * @return The azimuth angle of the velocity (from -180 to 180, inclusive)
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
     * <p>Returns the elevation angle of the robot with resepect to the
     * horizontal plane</p>
     * 
     * <p>Elevation is the angle between the horizontal component of the
     * velocity and the final velocity. By convention, upwards angles are
     * positive.</p>
     *
     * @return The elevation angle of the velocity (from -90 to 90, inclusive)
     */
    public Angle getElevationAngle() {
        if(this.cache_eleveationAngle == null) {
            double vZ_mps       = this.getZVelocity().in(Units.MetersPerSecond);
            double vHori_mps    = this.getHorizontalVelocity().in(Units.MetersPerSecond);
            this.cache_eleveationAngle = Units.Radians.of(Math.atan2(vZ_mps, vHori_mps));
        }
        return this.cache_eleveationAngle;
    }


    //
    // Transforms and Conversions
    //

    /**
     * Add another vector to this one and return the result
     *
     * @param other The other vector to add to this one
     * @return The sum of this vector and other
     */
    public Velocity3d plus(Velocity3d other) {
        return new Velocity3d(
            this.xVelocity.plus(other.xVelocity),
            this.yVelocity.plus(other.yVelocity),
            this.zVelocity.plus(other.zVelocity)
        );
    }

    /**
     * Subtract another vector from this one and return the result
     *
     * @param other The vector to subtract from this one
     * @return The difference of this vector from other
     */
    public Velocity3d minus(Velocity3d other) {
        return new Velocity3d(
            this.xVelocity.minus(other.xVelocity),
            this.yVelocity.minus(other.yVelocity),
            this.zVelocity.minus(other.zVelocity)
        );
    }

    /**
     * Rotate the vector by adding a given angle to the azimuth
     *
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


    //
    //  Overrides
    //

    @Override
    public int hashCode() {
        return this.xVelocity.hashCode() ^ this.yVelocity.hashCode() ^ this.zVelocity.hashCode();
    }


    @Override
    public String toString() {
        return String.format("Velocity3d(X: %s, Y: %s, Z: %s)", xVelocity, yVelocity, zVelocity);
    }

    /**
     * Get the string representation of this vector in spherical coordinates
     *
     * @return The string representation of this vector in spherical coordinates
     */
    public String toStringSpherical() {
        return String.format(
            "Velocity3d(Speed: %s m/s, Elevation: %s deg, Azimuth: %s deg)",
            this.getSpeed().in(Units.MetersPerSecond),
            this.getElevationAngle().in(Units.Degrees),
            this.getAzimuthAngle().in(Units.Degrees)
        );
    }

    @Override
    public boolean equals(Object obj) {
        if(Velocity3d.class.isInstance(obj)) {
            Velocity3d otherVelocity = (Velocity3d) obj;
            return
                this.xVelocity.equals(otherVelocity.getXVelocity()) &&
                this.yVelocity.equals(otherVelocity.getYVelocity()) &&
                this.zVelocity.equals(otherVelocity.getZVelocity());
        }
        return false;
    }
}
