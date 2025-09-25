import java.util.ArrayList;
import java.util.List;

public class VijPathing {

    // ---------- Small math helper classes ----------
    public static class Pose2d {
        public double x, y, heading; // heading in radians
        public Pose2d(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }

    public static class Waypoint {
        public double x, y;
        public double speed; // desired nominal speed at this waypoint (units/s or normalized)
        public Waypoint(double x, double y, double speed) {
            this.x = x; this.y = y; this.speed = speed;
        }
    }

    // ---------- Follower configuration (tune these) ----------
    private double LOOKAHEAD = 0.35;    // meters (or robot units). Increase for smoother larger curves.
    private double MAX_SPEED = 0.6;     // max forward speed (units consistent with odometry)
    private double kPathFollowing = 1.0; // multiplier for speed scaling near path end
    private double TRACK_WIDTH = 0.5;   // distance between left/right wheels (meters). Used to convert angular to wheel speeds.

    // completed path
    private List<Waypoint> path = new ArrayList<>();
    private int lastLookaheadIndex = 0;
    private double finishRadius = 0.1; // robot considered finished when within this radius to final waypoint

    public CustomPathFollower() {}

    public void setLookahead(double lookahead) { this.LOOKAHEAD = lookahead; }
    public void setMaxSpeed(double maxSpeed) { this.MAX_SPEED = maxSpeed; }
    public void setTrackWidth(double width) { this.TRACK_WIDTH = width; }
    public void setFinishRadius(double r) { this.finishRadius = r; }

    public void setPath(List<Waypoint> newPath) {
        this.path = new ArrayList<>(newPath);
        this.lastLookaheadIndex = 0;
    }

    public boolean isFinished(Pose2d pose) {
        if (path.isEmpty()) return true;
        Waypoint last = path.get(path.size() - 1);
        double dx = last.x - pose.x;
        double dy = last.y - pose.y;
        return Math.hypot(dx, dy) <= finishRadius;
    }

    // Main update: compute left/right wheel velocities
    // - pose: current robot pose (x,y,heading radians)
    // Returns double[2] = {leftVelocity, rightVelocity}
    public double[] update(Pose2d pose) {
        if (path.isEmpty()) return new double[] {0, 0};

        // 1) Find lookahead point on path
        LookaheadResult la = findLookaheadPoint(pose, LOOKAHEAD);
        // If cannot find a lookahead (i.e. at end), just aim at final point
        double goalX = la.goalX;
        double goalY = la.goalY;
        double targetSpeed = la.targetSpeed;

        // 2) Transform lookahead point into robot coordinate frame
        double dx = goalX - pose.x;
        double dy = goalY - pose.y;
        // robot heading zero means robot facing +x; rotate world -> robot
        double sinH = Math.sin(-pose.heading);
        double cosH = Math.cos(-pose.heading);
        double x_r = dx * cosH - dy * sinH;
        double y_r = dx * sinH + dy * cosH;

        // 3) Compute curvature to lookahead point (Pure Pursuit formula)
        // curvature kappa = (2 * y_r) / (L^2), where L is lookahead distance used.
        double L = Math.hypot(x_r, y_r);
        // protect divide-by-zero
        double curvature = 0.0;
        if (L > 1e-6) {
            curvature = (2.0 * y_r) / (L * L);
        }

        // 4) Compute linear and angular velocities
        double v = Math.min(targetSpeed, MAX_SPEED); // forward speed
        // Optionally, scale down v when curvature is large (sharp turns)
        double curvatureSlowdown = 1.0 / (1.0 + Math.abs(curvature) * 2.0); // tunable
        v *= curvatureSlowdown;

        double omega = curvature * v; // approximate angular velocity (rad/s)

        // 5) Convert to left/right wheel velocities for differential drive:
        // v_left = v - (omega * trackWidth/2)
        // v_right = v + (omega * trackWidth/2)
        double leftVel = v - omega * (TRACK_WIDTH / 2.0);
        double rightVel = v + omega * (TRACK_WIDTH / 2.0);

        // 6) Normalize if speeds exceed MAX_SPEED in magnitude (optional)
        double maxAbs = Math.max(Math.abs(leftVel), Math.abs(rightVel));
        double maxAllowed = Math.max(MAX_SPEED, 1e-6);
        if (maxAbs > maxAllowed) {
            leftVel = leftVel / maxAbs * maxAllowed;
            rightVel = rightVel / maxAbs * maxAllowed;
        }

        return new double[] { leftVel, rightVel };
    }

    // ---------- Lookahead point search ----------
    // Searches along path segments from lastLookaheadIndex to find the first point at distance LOOKAHEAD from robot position.
    // If not found, returns the final waypoint as goal.
    private static class LookaheadResult {
        double goalX, goalY, targetSpeed;
        LookaheadResult(double gx, double gy, double s) { goalX = gx; goalY = gy; targetSpeed = s; }
    }

    private LookaheadResult findLookaheadPoint(Pose2d pose, double lookahead) {
        // iterate segments
        for (int i = lastLookaheadIndex; i < path.size() - 1; i++) {
            Waypoint A = path.get(i);
            Waypoint B = path.get(i+1);

            // project robot onto segment and find intersections of circle (center=pose, radius=lookahead)
            // Parametric segment: P(t) = A + t*(B-A) for t in [0,1]
            double dx = B.x - A.x;
            double dy = B.y - A.y;

            double fx = A.x - pose.x;
            double fy = A.y - pose.y;

            double a = dx*dx + dy*dy;
            double b = 2*(fx*dx + fy*dy);
            double c = fx*fx + fy*fy - lookahead*lookahead;

            double discriminant = b*b - 4*a*c;
            if (discriminant < 0) {
                // no intersection with this segment
                continue;
            }

            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant) / (2*a);
            double t2 = (-b + discriminant) / (2*a);

            // prefer smallest t in [0,1] that is ahead of robot along the path
            double chosenT = Double.NaN;
            if (t1 >= 0 && t1 <= 1) chosenT = t1;
            else if (t2 >= 0 && t2 <= 1) chosenT = t2;

            if (!Double.isNaN(chosenT)) {
                double gx = A.x + dx * chosenT;
                double gy = A.y + dy * chosenT;
                double s = Math.min(A.speed, B.speed); // conservative speed
                lastLookaheadIndex = i; // speed optimization: start next search here
                return new LookaheadResult(gx, gy, s);
            }
        }

        // If no intersection found, goal is final waypoint
        Waypoint last = path.get(path.size() - 1);
        return new LookaheadResult(last.x, last.y, last.speed);
    }

    // ---------- Utility: create a simple straight-line path builder ----------
    public static List<Waypoint> simplePath(List<double[]> coords, double speed) {
        List<Waypoint> p = new ArrayList<>();
        for (double[] c : coords) {
            p.add(new Waypoint(c[0], c[1], speed));
        }
        return p;
    }

    // ---------- Example of usage & quick test harness ----------
    // The following shows how you'd call update(...) inside your opmode main loop:
    //
    // CustomPathFollower follower = new CustomPathFollower();
    // follower.setLookahead(0.35);
    // follower.setMaxSpeed(1.0);
    // follower.setTrackWidth(0.5);
    // List<Waypoint> myPath = new ArrayList<>();
    // myPath.add(new Waypoint(0,0, 0.6));
    // myPath.add(new Waypoint(1.2, 0.5, 0.6));
    // myPath.add(new Waypoint(2.0, 0.0, 0.4));
    // follower.setPath(myPath);
    //
    // Inside loop:
    // Pose2d currentPose = getOdometryPose(); // you must supply this
    // double[] wheelVels = follower.update(currentPose);
    // // Convert wheelVels (m/s) -> motor power or ticks, depending on your controllers.
    // // Example (normalized power):
    // double leftPower = wheelVels[0] / follower.MAX_SPEED;
    // double rightPower = wheelVels[1] / follower.MAX_SPEED;
    // // optionally clamp to [-1,1] and send to motors
}
