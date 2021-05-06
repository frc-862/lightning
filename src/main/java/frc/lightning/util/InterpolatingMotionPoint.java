package frc.lightning.util;

/**
 * A Double that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingMotionPoint implements Interpolable<InterpolatingMotionPoint> {
    public double position = 0.0;
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public double heading = 0.0;

    public InterpolatingMotionPoint(double p, double v, double a, double h) {
        position = p;
        velocity = v;
        acceleration = a;
        heading = h;
    }

    @Override
    public InterpolatingMotionPoint interpolate(InterpolatingMotionPoint other, double x) {
        double new_position = (other.position - position) * x + position;
        double new_velocity = (other.velocity - velocity) * x + velocity;
        double new_acceleration = (other.acceleration - acceleration) * x + acceleration;
        double new_heading = (other.heading - heading) * x + heading;

        return new InterpolatingMotionPoint(new_position, new_velocity, new_acceleration, new_heading);
    }

}