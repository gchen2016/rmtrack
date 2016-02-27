### Load the libs 

source("functions.r")

# the plots will be saved to this directory

make.plots <- function(imgdir, env) {
  runs <- load.and.preprocess(env)
  dir.create(file.path(imgdir, env), showWarnings = FALSE)
  
  for (nagents in unique(runs$nagents)) {
    print(nagents)
    
    traveltime.plot <- avgtravel.vs.disturbance (runs[runs$nagents==nagents & (runs$alg=="ALLSTOP" | runs$alg=="RMTRACK"),]) + theme(legend.position="none")
    ggsave(filename=paste(imgdir, env, "/traveltime-", nagents,"-robots.pdf", sep=""), width=3, height=3.2)
    
    traveltime.plot.w.legend <- traveltime.plot + theme(legend.position="bottom", legend.direction="horizontal", legend.box = "horizontal")
    ggsave(filename=paste(imgdir, env, "/traveltime-", nagents,"-robots-w-legend.pdf", sep=""), width=3, height=3.2)
    
    successrate.plot <- successrate.vs.disturbance(runs[runs$nagents==nagents & (runs$alg=="ORCA" | runs$alg=="RMTRACK"),]) + theme(legend.position="none")
    ggsave(filename=paste(imgdir, env, "/success-rate-", nagents,"-robots.pdf", sep=""), width=3, height=3.2)
    
    traveltime.common.plot <- avgtravel.vs.disturbance.common.only(runs[runs$nagents==nagents & (runs$alg=="ORCA" | runs$alg=="RMTRACK"),]) + theme(legend.position="none")
    ggsave(filename=paste(imgdir, env, "/traveltime-orca-", nagents,"-robots.pdf", sep=""), width=3, height=3.2)
  }
}

imgdir <- "plots/"

make.plots(imgdir=imgdir, env="ubremen-r27")
make.plots(imgdir=imgdir, env="warehouse-r25")
make.plots(imgdir=imgdir, env="empty-hall-r25")
