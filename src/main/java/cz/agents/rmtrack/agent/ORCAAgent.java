package cz.agents.rmtrack.agent;

import cz.agents.rmtrack.util.Disturbance;
import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import rvolib.*;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Polygon;
import tt.euclid2i.trajectory.TimePointArrayTrajectory;
import tt.euclid2i.util.Util;
import util.DesiredControl;
import util.GraphBasedOptimalPolicyController;

import java.util.*;

public class ORCAAgent extends Agent {
    static final Logger LOGGER = Logger.getLogger(ORCAAgent.class);

    private static final int MAX_NEIGHBORS = 50;
    private static final float NEIGHBOR_DIST = 500;
    private static final float TIME_HORIZON_AGENT = 5000;
    private static final float TIME_HORIZON_OBSTACLE = 1000;

    private static final double NEAR_GOAL_EPS = 0.0f;
    private final Random random;

    private RVOAgent rvoAgent;
    private HashMap<String, RVOAgent> neighbors;

    private KdTree kdTree;
    private ArrayList<RVOObstacle> obstacles;

    float maxSpeed;
    DesiredControl desiredControl;

    private static final long UNKNOWN = -1;

    private boolean showVis;

    private Collection<Region> ttObstaclesLessInflated;
    private double DesiredControlNodeSearchRadius;
    private boolean wasDisturbed = false;

    public ORCAAgent(int id, tt.euclid2d.Point start, tt.euclid2d.Point goal, Environment environment, DirectedGraph<Point, Line> planningGraph,
                     int agentBodyRadius, float maxSpeed, Disturbance disturbance, Random random, boolean showVis) {
        super(id, start, goal, agentBodyRadius, maxSpeed, disturbance);

        this.showVis = showVis;
        this.maxSpeed = maxSpeed;
        this.random = random;

        rvoAgent = new RVOAgent();

        rvoAgent.position_ = new Vector2(start);
        rvoAgent.velocity_ = new Vector2(0,0);

        rvoAgent.maxNeighbors_ = MAX_NEIGHBORS;
        rvoAgent.maxSpeed_ = maxSpeed;
        rvoAgent.neighborDist_ = NEIGHBOR_DIST;
        rvoAgent.radius_ = (float) Math.ceil(agentBodyRadius + 1);
        rvoAgent.timeHorizon_ = TIME_HORIZON_AGENT;
        rvoAgent.timeHorizonObst_ = TIME_HORIZON_OBSTACLE;

        rvoAgent.clearTrajectory();

        rvoAgent.id_ = id;

        rvoAgent.showVis = showVis;

        if (showVis) {
            rvoAgent.initVisualization();
        }

        LinkedList<Region> ttObstacles = new LinkedList<tt.euclid2i.Region>();
        ttObstacles.add(environment.getBoundary());
        ttObstacles.addAll(environment.getObstacles());

        this.ttObstaclesLessInflated = Util.inflateRegions(ttObstacles, agentBodyRadius-1);

        DesiredControlNodeSearchRadius = longestEdgeLength(planningGraph)+1; // Used to be: ((float) Math.ceil(agentBodyRadius * RADIUS_GRACE_MULTIPLIER) * 3) + 1; // approx. sqrt(2) * 2 * radius

        desiredControl = new GraphBasedOptimalPolicyController(planningGraph,
                goal.toPoint2i(), ttObstaclesLessInflated, maxSpeed,
                DesiredControlNodeSearchRadius, showVis);

        kdTree = new KdTree();

        // Add obstacles
        obstacles = new ArrayList<RVOObstacle>();

        for (tt.euclid2i.Region region : ttObstacles) {
            if (!(region instanceof Polygon)) {
                throw new RuntimeException("Only polygons are supported");
            }
            ArrayList<Vector2> obstacle = RVOUtil.regionToVectorList((Polygon) region);
            RVOUtil.addObstacle(obstacle, obstacles);
        }

        kdTree.buildObstacleTree(obstacles.toArray(new RVOObstacle[0]), this.obstacles);

        neighbors = new HashMap<String, RVOAgent>();
        neighbors.put(getName(), rvoAgent);
    }

    private double longestEdgeLength(DirectedGraph<Point, Line> planningGraph) {
        double longestEdgeLength = 0;
        for (Line edge : planningGraph.edgeSet()) {
            if (longestEdgeLength < edge.getDistance()) {
                longestEdgeLength = edge.getDistance();
            }
        }

        return longestEdgeLength;
    }

    public EvaluatedTrajectory getEvaluatedTrajectory(Point goal) {
        ArrayList<tt.euclidtime3i.Point> rvoTraj = new ArrayList<tt.euclidtime3i.Point>(rvoAgent.timePointTrajectory);
        tt.euclidtime3i.Point[] timePointArray = new tt.euclidtime3i.Point[rvoTraj.size()];
        for (int i=0; i<rvoTraj.size(); i++) {
            timePointArray[i] = new tt.euclidtime3i.Point(
                    rvoTraj.get(i).getPosition(),
                    rvoTraj.get(i).getTime());
        }

        double cost = RVOAgent.evaluateCost(timePointArray, goal);
        TimePointArrayTrajectory traj = new TimePointArrayTrajectory(timePointArray, cost);
        return traj;
    }

    private void doStep(int currentTime, float timeStep) {

        LOGGER.trace(getName() + " -- doStep");

        updateNeighborsFromBlackboard();

        setPreferredVelocity(timeStep);

        RVOAgent[] rvoAgents = neighbors.values().toArray(new RVOAgent[neighbors.values().size()]);

        kdTree.clearAgents();
        kdTree.buildAgentTree(rvoAgents);

        rvoAgent.computeNeighbors(kdTree);

        Vector2 newVelocity = rvoAgent.computeNewVelocity(timeStep);

        // Add small random disturbance
        final float RANDOM_DISTURBANCE = 0.001f;
        newVelocity = new Vector2(
                newVelocity.x() - RANDOM_DISTURBANCE/2 + RANDOM_DISTURBANCE*random.nextFloat(),
                newVelocity.y() - RANDOM_DISTURBANCE/2 + RANDOM_DISTURBANCE*random.nextFloat());

        if (isDisturbed(currentTime)) {
            newVelocity = new Vector2(0,0);
            wasDisturbed = true;
        } else {
            wasDisturbed = false;
        }

        rvoAgent.update(timeStep, newVelocity);

        PositionBlackboard.recordNewPosition(getName(), rvoAgent.id_, rvoAgent.position_.toPoint2d(), rvoAgent.velocity_.toVector2d(), rvoAgent.radius_);
    }

    private void setPreferredVelocity(float timeStep) {
        Vector2 currentPosition = rvoAgent.position_;
        double distanceToGoal = currentPosition.toPoint2d().distance(goal);

        //if (currentPosition.toPoint2i().distance(getCurrentGoal()) < NEAR_GOAL_EPS) {
        //rvoAgent.setPrefVelocity(new Vector2(0, 0));
        //} else {
        tt.euclid2d.Vector desiredVelocity = desiredControl.getDesiredControl(rvoAgent.position_.toPoint2d());
        assert !Double.isNaN(desiredVelocity.x) && !Double.isNaN(desiredVelocity.y);
        double desiredSpeed = desiredVelocity.length();
        tt.euclid2d.Vector desiredDir = new tt.euclid2d.Vector(desiredVelocity);
        if (desiredDir.length() != 0) {
            desiredDir.normalize();
        }

        // Adjust if the agent is near the goal
        if (distanceToGoal <= timeStep * desiredSpeed) {
            // goal will be reached in the next time step
            double speed = distanceToGoal / timeStep;
            desiredVelocity = desiredDir;
            desiredVelocity.scale(speed);
        }

        rvoAgent.setPrefVelocity(new Vector2(desiredVelocity));
        //}
    }

    protected void updateNeighborsFromBlackboard() {
        for (Map.Entry<String, PositionBlackboard.Record> entry : PositionBlackboard.getRecords().entrySet()) {
            if (!entry.getKey().equals(getName())) {
                RVOAgent neighborRVOAgent = new RVOAgent();
                neighborRVOAgent.id_ = entry.getValue().agentId;
                neighborRVOAgent.position_ = new Vector2(entry.getValue().position);
                neighborRVOAgent.velocity_ = new Vector2(entry.getValue().velocity);
                neighborRVOAgent.radius_ = (float) entry.getValue().radius;
                neighbors.put(entry.getKey(), neighborRVOAgent);
            }
        }
    }

    public tt.euclid2d.Vector getCurrentVelocity() {
        return rvoAgent.velocity_.toVector2d();
    }


    @Override
    public void update(int t_current_ms, int t_next_ms, List<Agent> agents) {

        if (!isAtGoal()) {
            travelTime = t_current_ms;
        }

        float timeStep = (float) (t_next_ms - t_current_ms);
        doStep(t_current_ms, timeStep);
    }

    @Override
    public tt.euclid2d.Point getPosition() {
        return rvoAgent.position_.toPoint2d();
    }

    @Override
    public boolean wasDisturbed() {
        return wasDisturbed;
    }

    private String getName() {
        return "a"+id;
    }
}
