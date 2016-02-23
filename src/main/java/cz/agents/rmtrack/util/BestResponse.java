package cz.agents.rmtrack.util;

import java.awt.Color;
import java.util.Collection;
import java.util.Random;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;

import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.VisLayer;
import org.jgrapht.util.heuristics.PerfectHeuristic;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.SegmentedTrajectory;
import tt.euclid2i.discretization.L2Heuristic;
import tt.euclid2i.discretization.LazyGrid;
import tt.euclid2i.discretization.ObstacleWrapper;
import tt.euclid2i.discretization.ToGoalEdgeExtension;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.trajectory.LineSegmentsConstantSpeedTrajectory;
import tt.euclid2i.trajectory.StraightSegmentTrajectory;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclid2i.vis.RegionsLayer.RegionsProvider;
import tt.euclidtime3i.discretization.ConstantSpeedTimeExtension;
import tt.euclidtime3i.discretization.ControlEffortWrapper;
import tt.euclidtime3i.discretization.FreeOnTargetWaitExtension;
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.sipp.SippEdge;
import tt.euclidtime3i.sipp.SippGoal;
import tt.euclidtime3i.sipp.SippHeuristic;
import tt.euclidtime3i.sipp.SippNode;
import tt.euclidtime3i.sipp.SippUtils;
import tt.euclidtime3i.sipp.SippWrapper;
import tt.euclidtime3i.sipp.intervals.Interval;
import tt.euclidtime3i.sipprrts.DynamicObstaclesImpl;
import tt.euclidtime3i.vis.TimeParameterProjectionTo2d;
import tt.vis.GraphLayer;
import tt.vis.TimeParameterHolder;

public class BestResponse {

    private final static Logger LOGGER = Logger.getLogger(BestResponse.class);
	private static final boolean DEBUG_VIS = false;

	public static EvaluatedTrajectory computeBestResponse(final Point start,
														  final Point goal,
                                                          final float maxSpeed,
														  final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
														  final Collection<tt.euclid2i.Region> staticObstacles,
														  final Collection<tt.euclidtime3i.Region> dynamicObstacles,
                                                          final int maxTime, final int timeStep) {

        ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line> adaptedSpatialGraph
                = new ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line>(spatialGraph, staticObstacles);

        DirectedGraph<Point, Line> explicitSpatialGraph = adaptedSpatialGraph.generateFullGraph(start);

        final HeuristicToGoal<tt.euclid2i.Point> spatialHeuristic = new PerfectHeuristic<Point, Line>(explicitSpatialGraph, goal);
        final HeuristicToGoal<tt.euclidtime3i.Point> spaceTimeHeuristic = new HeuristicToGoal<tt.euclidtime3i.Point>() {
            @Override
            public double getCostToGoalEstimate(tt.euclidtime3i.Point current) {
                return spatialHeuristic.getCostToGoalEstimate(current.getPosition()) / (double) maxSpeed;
            }
        };

		VisLayer graphLayer;
		VisLayer sobstLayer;
		VisLayer dobstLayer;

		if (DEBUG_VIS) {
			// --- debug visio --- begin
			//visualize the graph
			graphLayer = GraphLayer.create(
					new GraphLayer.GraphProvider<tt.euclid2i.Point, tt.euclid2i.Line>() {
						@Override
						public Graph<tt.euclid2i.Point, tt.euclid2i.Line> getGraph() {
							return explicitSpatialGraph;
						}
					}, new tt.euclid2i.vis.ProjectionTo2d(), Color.CYAN,
					Color.CYAN, 1, 4);

			sobstLayer = RegionsLayer.create(new tt.euclid2i.vis.RegionsLayer.RegionsProvider() {
				@Override
				public Collection<? extends Region> getRegions() {
					return staticObstacles;
				}
			}, Color.ORANGE);

			dobstLayer = tt.euclidtime3i.vis.RegionsLayer.create(new tt.euclidtime3i.vis.RegionsLayer.RegionsProvider() {

				@Override
				public Collection<tt.euclidtime3i.Region> getRegions() {
					return dynamicObstacles;
				}
			}, new TimeParameterProjectionTo2d(TimeParameterHolder.time), Color.BLACK, null);


			VisManager.registerLayer(graphLayer);
			VisManager.registerLayer(sobstLayer);
			VisManager.registerLayer(dobstLayer);

			// --- debug visio --- end
		}

		// time-extension
		DirectedGraph<tt.euclidtime3i.Point, Straight> motions
				= new ConstantSpeedTimeExtension(explicitSpatialGraph, maxTime, new float[]{maxSpeed}, dynamicObstacles, timeStep, timeStep);

		motions = new FreeOnTargetWaitExtension(motions, goal);
		motions = new ControlEffortWrapper(motions, 0.01);

        Goal<tt.euclidtime3i.Point> goalCond = new Goal<tt.euclidtime3i.Point>() {
            @Override
            public boolean isGoal(tt.euclidtime3i.Point current) {
                return current.getPosition().equals(goal) && current.getTime() > (maxTime - timeStep - 1); // last space-time node might not be placed at MAX_TIME
            }
        };
        AStarShortestPathSimple<tt.euclidtime3i.Point, Straight> astar
                = new AStarShortestPathSimple<>(motions, spaceTimeHeuristic, new tt.euclidtime3i.Point(start, 0), goalCond);

        long startedAt = System.currentTimeMillis();
        GraphPath<tt.euclidtime3i.Point, Straight> path = astar.findPathRuntimeLimit(Integer.MAX_VALUE, Integer.MAX_VALUE);

        long runtime = System.currentTimeMillis() - startedAt;
        LOGGER.debug("Planning finshed in " + runtime + "ms; "
                + astar.getIterationCount() + " iterations; "
                + (int) (astar.getIterationCount() / (runtime / 1000.0)) + " it/sec "
                + " path-length:" + ((path != null) ? path.getEdgeList().size() : "NONE"));

		if (path != null) {
			if (DEBUG_VIS) {
				VisManager.unregisterLayer(graphLayer);
				VisManager.unregisterLayer(sobstLayer);
				VisManager.unregisterLayer(dobstLayer);
			}
			return new StraightSegmentTrajectory(path, maxTime);
		} else {
			return null;
		}
	}

	public static EvaluatedTrajectory computeBestResponseSIPP(final Point start,
															  final Point goal,
															  final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
															  Collection<tt.euclidtime3i.Region> avoid, final int maxTime) {

		final SegmentedTrajectory[] trajArr = new SegmentedTrajectory[avoid.size()];
		int[] radiuses = new int[trajArr.length];

		int i = 0;
		for (tt.euclidtime3i.Region region : avoid) {
			assert region instanceof MovingCircle;
			MovingCircle mc = (MovingCircle) region;
			assert mc.getTrajectory() instanceof MovingCircle;
			trajArr[i] = (SegmentedTrajectory) mc.getTrajectory();
			radiuses[i] = mc.getRadius();
			i++;
		}

		DynamicObstaclesImpl dynamicEnv = new DynamicObstaclesImpl(trajArr, radiuses, maxTime);

		System.out.println("Creating SIPP Wrapper...");

		SippWrapper wrapper = new SippWrapper(spatialGraph, dynamicEnv, 0, 1, 2, maxTime);
		SippNode startSipp = new SippNode(start, Interval.toInfinity(0), 0);
		SippHeuristic heuristic = new SippHeuristic(new L2Heuristic(goal), 1);
		SippGoal goalSipp = new SippGoal(goal, maxTime);

		System.out.println("..Done \nStarting A* search...");

		GraphPath<SippNode, SippEdge> path = AStarShortestPathSimple.findPathBetween(wrapper, heuristic, startSipp, goalSipp);

		if (path != null) {
			final SegmentedTrajectory trajectory = SippUtils.parseTrajectory(path, maxTime);
			return trajectory;
		} else {
			return null;
		}
	}

	public static EvaluatedTrajectory computeShortestPath(
			final Point start,
			final Point goal,
			final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
			HeuristicToGoal<Point> heuristic,
			Collection<tt.euclid2i.Region> staticObstacles) {

		final ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line> adaptedSpatialGraph
				= new ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line>(spatialGraph, staticObstacles);

		GraphPath<Point, Line> path = AStarShortestPathSimple.findPathBetween(adaptedSpatialGraph,
				heuristic,
				start,
				goal);

		return new LineSegmentsConstantSpeedTrajectory<Point, Line>(0, path, 1, (int) path.getWeight());
	}

}

