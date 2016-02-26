package cz.agents.rmtrack;
import cz.agents.alite.simulation.vis.SimulationControlLayer;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.rmtrack.agent.Agent;
import cz.agents.rmtrack.agent.TrackingAgent;
import cz.agents.rmtrack.util.Disturbance;
import org.apache.log4j.Logger;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Trajectory;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.jointeuclid2ni.probleminstance.TrajectoryCoordinationProblemXMLDeserializer;
import tt.jointeuclid2ni.probleminstance.VisUtil;
import tt.jointeuclid2ni.solver.Parameters;
import tt.util.AgentColors;
import tt.util.Args;
import tt.vis.FastTrajectoriesLayer;
import tt.vis.FastTrajectoriesLayer.ColorProvider;
import tt.vis.FastTrajectoriesLayer.TrajectoriesProvider;
import tt.vis.LabeledCircleLayer;

import java.awt.*;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;


public class ScenarioCreator {
	
	/* 
	 * Units:
	 * time: 1 time unit = 1ms; 
	 * distance: 1 distance unit = depending on the map, 2cm typically. 
	 * speed: in du/ms (distance units / millisecond), typically 0.05 du/ms represents roughly 1m/1s 
	 */

	////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
    	createFromArgs(args);
    }

    ///////////////////////////////////////////////////////////////////////
    
    static long simulationStartedAt;
    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);

    enum Method {
        ALLSTOP,
        RMTRACK,
        ORCA}


    private static EarliestArrivalProblem problem;

    public static void createFromArgs(String[] args) {
    	simulationStartedAt = System.currentTimeMillis();
    	Parameters params = new Parameters();
    	 	
    	String xml = Args.getArgumentValue(args, "-problemfile", true);
    	String methodStr = Args.getArgumentValue(args, "-method", true);
    	String maxTimeStr = Args.getArgumentValue(args, "-maxtime", true);
    	params.maxTime = Integer.parseInt(maxTimeStr);
    	String timeStepStr = Args.getArgumentValue(args, "-timestep", true);
    	params.timeStep = Integer.parseInt(timeStepStr);
    	params.showVis = Args.isArgumentSet(args, "-showvis");
    	params.verbose = Args.isArgumentSet(args, "-verbose");
    	String timeoutStr = Args.getArgumentValue(args, "-timeout", false);
        params.summaryPrefix = Args.getArgumentValue(args, "-summaryprefix", false, "");
        params.activityLogFile = Args.getArgumentValue(args, "-activitylog", false, null);
        String bgImgFileName = Args.getArgumentValue(args, "-bgimg", false, null);
        String simSpeedStr = Args.getArgumentValue(args, "-simspeed", false, "1");
        params.simSpeed = Double.parseDouble(simSpeedStr);

        String disturbanceProbStr = Args.getArgumentValue(args, "-dprob", false, "10");

        String[] parts = disturbanceProbStr.split("s", -1);
        params.disturbanceProbs = new double[parts.length];
        for (int i = 0; i < parts.length; i++) {
            params.disturbanceProbs[i] = Integer.parseInt(parts[i]) / 100.0;
        }

        String disturbanceSeedStr = Args.getArgumentValue(args, "-dseed", false, "1");
        params.disturbanceSeed = Integer.parseInt(disturbanceSeedStr);

        String disturbanceQuantStr = Args.getArgumentValue(args, "-dquant", false, "1000");
        params.disturbanceQuantum = Integer.parseInt(disturbanceQuantStr);

		File file = new File(xml);
	    params.fileName = file.getName();
	    
	    // Load the PNG image as a background, if provided
	    if (bgImgFileName != null) {
		    File bgImgFile = new File(bgImgFileName);
		    if (bgImgFile.exists()) {
		    	params.bgImageFile = bgImgFile;
		    }        	
        }

	    try {
			problem = TrajectoryCoordinationProblemXMLDeserializer.deserialize(new FileInputStream(file));
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}

	    Method method = Method.valueOf(methodStr);
	    
	    params.runtimeDeadlineMs = 3600*1000; /* default timeout is 1 hour */
	    if (timeoutStr != null) {
	    	int timeout = Integer.parseInt(timeoutStr);
	    	params.runtimeDeadlineMs = timeout;
	    	killAt(System.currentTimeMillis() + timeout, params.summaryPrefix, params.noOfClusters);
	    }

        switch (method) {
            case ALLSTOP:
                solveTracking(problem, TrackingAgent.TrackingMethod.ALLSTOP, params);
                break;

            case RMTRACK:
                solveTracking(problem, TrackingAgent.TrackingMethod.RMTRACK, params);
                break;

            case ORCA:
                solveORCA(problem, params);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }


    }

    private static void killAt(final long killAtMs, final String summaryPrefix, final int clusters) {
    	Thread t = new Thread() {
			@Override
			public void run() {
				while (System.currentTimeMillis() < killAtMs) {
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {}
				}
				printSummary(summaryPrefix, Status.TIMEOUT, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
				System.exit(0);
			}
    	};
    	t.start();
	}

	private static void solveTracking(final EarliestArrivalProblem problem,
                                      TrackingAgent.TrackingMethod trackingMethod, final Parameters params) {


        // find minimum lengths
        int[] durationsOverShortestPath = Util.computeDurationOverShortestPath(problem);

        // find trajectories
        final EvaluatedTrajectory[] trajs = Util.findCollisionFreeTrajs(problem, params.timeStep, params.maxTime);

        for (double disturbanceProb : params.disturbanceProbs) {

            if (params.showVis) {
                VisUtil.initVisualization(problem.getEnvironment(), "RMTRACK", params.bgImageFile, params.timeStep/2);
                VisUtil.visualizeEarliestArrivalProblem(problem);
            }

            Disturbance disturbance = new Disturbance((float) disturbanceProb, params.disturbanceQuantum, params.disturbanceSeed, problem.nAgents());
            final int[] lowerBoundDurations = Util.findLowerBoundDuration(problem, trajs, disturbance);

            List<Agent> agents = new LinkedList<>();
            for (int i=0; i < problem.nAgents(); i++) {
                agents.add(i,
                        new TrackingAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                trajs[i],
                                disturbance,
                                trackingMethod));
            }

            // simulate execution
            simulate(problem, agents, params);
            printStatistics(agents, disturbanceProb, durationsOverShortestPath, lowerBoundDurations, params);
            VisManager.unregisterLayers();
        }

        System.exit(0);
    } 
	
	private static void solveORCA(final EarliestArrivalProblem problem, final Parameters params) {
//        simulate(problem, new AgentFactory() {
//            @Override
//            public Agent createAgent(String name, int i, Point start, int nTasks,
//                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius, float speed) {
//
//				Agent agent = new ORCAAgent(name, start, nTasks, env, planningGraph, agentBodyRadius, speed, params.maxTime, params.timeStep, params.showVis, params.random);
//				return agent;
//            }
//        }, params);
    }    

    private static void simulate(final EarliestArrivalProblem problem, List<Agent> agents, final Parameters params) {
    	
    	simulationStartedAt = System.currentTimeMillis();
        initAgentVisualization(agents, params.timeStep);

        int SIMULATION_STEP_MS = 100;
        int SIMULATE_UNTIL = 600000;

        class SimulationControl {
            public int simulatedTimeMs = 0;
            public float simSpeed = 1;
            public boolean running = true;
        }

        final SimulationControl simControl = new SimulationControl();

        // Simulation Control Layer
        VisManager.registerLayer(SimulationControlLayer.create(new SimulationControlLayer.SimulationControlProvider() {

            @Override
            public void setSpeed(float f) {simControl.simSpeed = f;}

            @Override
            public void setRunning(boolean running) {simControl.running = running;}

            @Override
            public boolean isRunning() { return simControl.running; }

            @Override
            public double getTime() { return (simControl.simulatedTimeMs / 1000.0); }

            @Override
            public float getSpeed() { return simControl.simSpeed; }
        }));

        while (!allDone(agents) && simControl.simulatedTimeMs < SIMULATE_UNTIL){
            if (simControl.running == true) {

                for (Agent agent : agents) {
                    agent.update(simControl.simulatedTimeMs, simControl.simulatedTimeMs + SIMULATION_STEP_MS, agents);
                }

                if (params.showVis) {
                    try {
                        Thread.sleep((int) ((double) SIMULATION_STEP_MS * (double) simControl.simSpeed));
                    } catch (InterruptedException e) {
                    }
                }

                simControl.simulatedTimeMs += SIMULATION_STEP_MS;
            } else {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) { }
            }
        }
    }

    private static void printStatistics(List<Agent> agents, double disturbance,  int[] shortestPathDurations, int[] lbDurations, Parameters params) {
        long baseTimeSum = 0;
        long lbTimeSum = 0;
        long travelTimeSum = 0;
        long prolongSum = 0;
        long prolongSumSq = 0;
        long makespanAbs = 0;
        long makespanProlong = 0;

        for (int i=0; i<agents.size(); i++) {
            Agent agent = agents.get(i);
            baseTimeSum += shortestPathDurations[i];
            lbTimeSum += lbDurations[i];
            travelTimeSum += agent.travelTime;
            long prolongation = agent.travelTime - lbDurations[i];
            prolongSum += prolongation;
            prolongSumSq += prolongation * prolongation;

            if (agent.travelTime > makespanAbs)
                makespanAbs = agent.travelTime;

            if ((agent.travelTime - lbDurations[i]) > makespanProlong)
                makespanProlong = (agent.travelTime - lbDurations[i]);
        }

        long n = agents.size();

        long avgBaseTime = avg(baseTimeSum, n);
        long avgLbTime = avg(lbTimeSum, n);
        long avgTravelTime = avg(travelTimeSum, n);

        // status;dprob;dquant;dseed;avgBase;avgLb;avgTravel;prolongSum;prolongSumSq;makespanAbs;makespanRel

        printSummary(params.summaryPrefix, Status.SUCCESS, disturbance, params.disturbanceQuantum, params.disturbanceSeed,
                     avgBaseTime, avgLbTime, avgTravelTime, prolongSum, prolongSumSq, makespanAbs, makespanProlong);
    }
    
    private static boolean allDone(List<Agent> agents) {
        final double EPS = 0.9;
    	for (final Agent agent : agents) {
            agent.isAtGoal();
    		if (!agent.isAtGoal()) {
                return false;
            }
    	}
    	return true;
	}


	private static void initAgentVisualization(final List<Agent> agents, int timeStep) {
        // trajectories
        VisManager.registerLayer(
                KeyToggleLayer.create("t", true,
                        FastTrajectoriesLayer.create(new TrajectoriesProvider() {

                            @Override
                            public Trajectory[] getTrajectories() {
                                Trajectory[] trajsArr = new Trajectory[agents.size()];

                                for (int i = 0; i < trajsArr.length; i++) {
                                    Agent agent = agents.get(i);
                                    if (agent instanceof TrackingAgent) {
                                        trajsArr[i] = ((TrackingAgent) agent).getTrajectory();
                                    }
                                }
                                return trajsArr;
                            }
                        },new ColorProvider() {

                            @Override
                            public Color getColor(int i) {
                                return AgentColors.getColorForAgent(i);
                            }
                        }, 3, timeStep)));
		
        // positions
        VisManager.registerLayer(
        	KeyToggleLayer.create("b", true, 
        	LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2d.Point>() {

             @Override
             public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> getLabeledCircles() {
                 LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>>();
                 for (int i = 0; i < agents.size(); i++) {
                     tt.euclid2d.Point pos = agents.get(i).getPosition();

                     Color fillColor = AgentColors.getColorForAgent(i);
                     if (agents.get(i).wasDisturbed()) {
                         fillColor = Color.black;
                     }

                     Color textColor = Color.WHITE;

                     if (agents.get(i) instanceof TrackingAgent) {
                         TrackingAgent agent = (TrackingAgent) agents.get(i);
                         if (agent.isCurrentlyWaiting()) {
                             textColor = Color.DARK_GRAY;
                         }
                     }

                	 list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>(pos,
                             (int) agents.get(i).getRadius(), "" + i  , AgentColors.getColorForAgent(i),
                             fillColor ,
                             textColor));
                 }

                 return list;
             }

         }, new tt.euclid2d.vis.ProjectionTo2d())));
 	}

    enum Status {SUCCESS, FAIL, TIMEOUT}

	static long avg(long sum, long n) {
		return sum / n;
	}

    /* status;dprob;dquant;dseed;avgBase;avgLb;avgTravel;prolongSum;prolongSumSq;makespanAbs;makespanRel */
    private static void printSummary(String prefix, Status status, double dprob, int dquant, int dseed,
                                     long avgBase, long avgLb, long avgTravel, long prolongSum, long prolongSumSq,
                                     long makespanAbs,
                                     long makespanRel) {
        System.out.println(prefix +
                status.toString() + ";" +
                dprob + ";" +
                dquant + ";" +
                dseed + ";" +
                avgBase + ";" +
                avgLb + ";" +
                avgTravel + ";" +
                prolongSum + ";" +
                prolongSumSq + ";" +
                makespanAbs + ";" +
                makespanRel + ";");
    }

	static long sd(long sumSq, long mean, long n) {
		double var = (sumSq/n) - (mean*mean); 
		return (long) Math.round(Math.sqrt(var));
	}

}
