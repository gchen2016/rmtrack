#!/bin/bash

function generate_instance_set {
    denvxml=$1
    instancefolder=$2
    radius=$3
    gridedgelen=$4
    maxtime=$5
    agents=$6
    maxspeed="0.05"
    timestep=`echo "import math;print(int(math.ceil($gridedgelen/(2*$maxspeed))))" | python`

    echo "Preparing instanceset $instancesetname. Will use timestep $timestep."

    mkdir -p $instancefolder
    rm $instancefolder/*
    cp prepare.sh $instancefolder/

    instance=0
    for nagents in $agents
    do
        for seed in {1..10}
        do
            let instance=instance+1
            # create a problem instance file
            instancename="$instance"
            instancefile=$instancefolder/$instancename.xml

            ## ConflictGenerator
            java -XX:+UseSerialGC -cp solver.jar -Dlog4j.configuration="file:$PWD/log4j.custom" tt.jointeuclid2ni.probleminstance.generator.GenerateEAInstance -env $denvxml -nagents $nagents -radius $radius -maxspeed $maxspeed -seed $seed -sgnooverlap -outfile $instancefile

            algs="ORCA RMTRACK"

            for alg in $algs
            do
                summaryprefix="$envname;$instance;$nagents;$radius;$seed;$timestep;$maxtime;$alg;"
                echo -method $alg -problemfile $instancefile -ntasks $ntasks -timestep $timestep -maxtime $maxtime -timeout $maxtime -seed $seed -summaryprefix "$summaryprefix" >> $instancefolder/data.in
            done

            echo Finished instance no $instance. Agents: $nagents. Seed: $seed.
        done
    done
    echo "env;instance;nagents;radius;seed;timestep;maxtime;alg;status;disturbance;sumtime;" > $instancefolder/head
    echo Done. Created $instance instances at $envname environment. Instances stored in $instancefolder.
}

# ubremen
generate_instance_set d-envs/ubremen-r27-docks.xml instances/ubremen-r27 27 65 600000 "1 10 35"

# warehouse
#generate_instance_set d-envs/warehouse-r25-docks.xml instances/warehouse-r25 25 54 600000 "1 10 50"

# emptyhall
#generate_instance_set d-envs/empty-hall-r25-docks.xml instances/empty-hall-r25 20 54 600000 "1 10 50"
