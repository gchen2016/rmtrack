### Load the libs 

source("functions.r")

# the plots will be saved to this directory

runs <- load.and.preprocess("ubremen-r27") # "ubremen-r27 warehouse-r25" "empty-hall-r25"
ggplot(runs[runs$nagents==20 & runs$status=="SUCCESS",], aes(x=dprob, y=avgTravel/1000, color=alg, shape=alg)) + geom_point(position=position_dodge(0.02)) + ylim(0,600)

succrate <- ddply(runs[runs$nagents==35,], .(dprob, alg), summarise,  
                nsolved = sum(status == "SUCCESS"),
                succrate = sum(status == "SUCCESS") / length(status),
                mean = mean(avgTravel, na.rm=TRUE))

ggplot(succrate, aes(x=dprob, y=succrate, color=alg)) + geom_line()

