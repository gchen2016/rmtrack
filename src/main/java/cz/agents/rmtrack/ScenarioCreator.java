package cz.agents.rmtrack;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.rmtrack.agent.Agent;
import cz.agents.rmtrack.agent.RMTrackAgent;
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
import java.util.Random;


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

        String disturbanceProbStr = Args.getArgumentValue(args, "-dprob", false, "0.1");
        params.disturbanceProb = Double.parseDouble(disturbanceProbStr);

        String disturbanceSeedStr = Args.getArgumentValue(args, "-dseed", false, "1");
        params.disturbanceSeed = Integer.parseInt(disturbanceSeedStr);

        String seedStr = Args.getArgumentValue(args, "-seed", true);
    	params.random = new Random(Integer.parseInt(seedStr));
    	
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
	    
    	create(problem, method, params);
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
				printSummary(summaryPrefix, Status.TIMEOUT, -1, -1);
				System.exit(0);
			}
    	};
    	t.start();
	}


	public static void create(EarliestArrivalProblem problem, Method method, final Parameters params) {
		
        if (params.showVis) {
            VisUtil.initVisualization(problem, "Trajectory Tools ("+method.toString()+")", params.bgImageFile, params.timeStep/2);
            VisUtil.visualizeEarliestArrivalProblem(problem);
        }

        Disturbance disturbance = new Disturbance((float) params.disturbanceProb, 1000, params.disturbanceSeed, problem.nAgents());
        
        switch (method) {
        
			case RMTRACK:
	            solveRMTrack(problem, disturbance, params);
	            break;

	        case ORCA:
                solveORCA(problem, params);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }
    }

	private static void solveRMTrack(final EarliestArrivalProblem problem, Disturbance disturbance, final Parameters params) {

        // find trajectories
        final EvaluatedTrajectory[] trajs = RPP.solve(problem, params.timeStep, params.maxTime);

        List<Agent> agents = new LinkedList<>();
        for (int i=0; i < problem.nAgents(); i++) {
            agents.add(i,
                    new RMTrackAgent(i,
                        problem.getStart(i).toPoint2d(),
                        problem.getTarget(i).toPoint2d(),
                        problem.getBodyRadius(i),
                        problem.getMaxSpeed(i),
                        trajs[i],
                        disturbance));
        }

		// simulate execution
		simulate(problem, agents, params);
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
        int simulatedTimeMs = 0;
        while (!allDone(agents) && simulatedTimeMs < SIMULATE_UNTIL){
            simulatedTimeMs += SIMULATION_STEP_MS;

            for (Agent agent: agents){
                agent.update(simulatedTimeMs, simulatedTimeMs+SIMULATION_STEP_MS, agents);
            }

            try {
                Thread.sleep(/*SIMULATION_STEP_MS*/ 1);
            } catch (InterruptedException e) {}
        }

    	long sum = 0;
    	long sumSq = 0;

        for (Agent agent : agents) {
        	sum +=  agent.goalReachedSum;
        	sumSq += agent.goalReachedSumSq;
		}
        
        long n = agents.size();
        
        long avgGoalReachedTime = avg(sum, n);
        long varGoalReachedTime = sd(sumSq, avgGoalReachedTime, n);
        // avgBase;varBase;avgWait;varWait;avgPlan;varPlan;avgPWindow;varPWindow;avgProlongT;varProlongT;avgProlongR;varProlongR;makespan

		printSummary(params.summaryPrefix, Status.SUCCESS, avgGoalReachedTime, varGoalReachedTime);
    }
    
    private static boolean allDone(List<Agent> agents) {
    	for (final Agent agent : agents) {
    		if (!agent.isDone())
    			return false;
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
                                    if (agent instanceof RMTrackAgent) {
                                        trajsArr[i] = ((RMTrackAgent) agent).getTrajectory();
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
                     if (agents.get(i).isCurrentlyDisturbed()) {
                         fillColor = Color.black;
                     }

                     Color textColor = Color.WHITE;

                     if (agents.get(i) instanceof RMTrackAgent) {
                         RMTrackAgent agent = (RMTrackAgent) agents.get(i);
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

    private static void printSummary(String prefix, Status status,
                                     long avgBase, long varBase) {
        System.out.println(prefix + status.toString()
                + ";" + avgBase  + ";" + varBase);
    }

	static long sd(long sumSq, long mean, long n) {
		double var = (sumSq/n) - (mean*mean); 
		return (long) Math.round(Math.sqrt(var));
	}

}
