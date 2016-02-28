package cz.agents.rmtrack.agent;

import tt.euclid2d.Point;
import tt.euclid2d.Vector;
import tt.euclid2i.Trajectory;
import util.DesiredControl;

public class TrajectoryFollowingControl implements DesiredControl {

    final static int LOOKAHEAD = 500;

    Trajectory traj;
    int currentTime;
    double maxSpeed;

    public TrajectoryFollowingControl(Trajectory traj, double maxSpeed) {
        assert traj != null;
        this.traj = traj;
        this.maxSpeed = maxSpeed;
    }

    public void setCurrentTime(int currentTime) {
        this.currentTime = currentTime;
    }

    @Override
    public Vector getDesiredControl(Point currentPosition) {
        return getVelocity(traj, currentPosition, currentTime, maxSpeed);
    }

    public Vector getVelocity(Trajectory traj, Point currentPosition, int currentTime, double maxSpeed) {
        if ((currentTime + LOOKAHEAD) < traj.getMaxTime()) {

            Point carrot = traj.get(currentTime + LOOKAHEAD).toPoint2d();

            Vector vel = new Vector(carrot);
            vel.sub(currentPosition);
            vel.scale(1 / (double) LOOKAHEAD);

            if (vel.length() > maxSpeed) {
                vel.normalize();
                vel.scale(maxSpeed);
            }

            return vel;
        } else {
            return new Vector(0,0);
        }
    }

    public Trajectory getTraj() {
        return traj;
    }
}
