package cz.agents.rmtrack.util;

import java.util.Random;

public class Disturbance {

    private final boolean[][] disturbances;
    float probability;
    int timeQuantum;
    int seed;

    public Disturbance(float probability, int timeQuantum, int seed, int nAgents) {
        this.probability = probability;
        this.timeQuantum = timeQuantum;
        this.seed = seed;

        Random random = new Random(seed);

        int len = 10000;
        this.disturbances = new boolean[nAgents][len];
        for (int i=0; i<nAgents; i++) {
            for (int t=0; t<len; t++)
            disturbances[i][t] = random.nextFloat() < probability;
        }

    }

    public boolean isDisturbed(int i, int time){
        int tq = time / timeQuantum;
        return disturbances[i][tq];
    }
}
