### Load the libs 

source("functions.r")

# the plots will be saved to this directory

make.plots <- function(imgdir, env) {
  runs <- load.and.preprocess(env)
  dir.create(file.path(imgdir, env), showWarnings = FALSE)
  
  for (nagents in unique(runs$nagents)) {
    print(nagents)
    traveltime.plot <- avgtravel.vs.disturbance (runs[runs$nagents==nagents,]) + theme(legend.position="none")
    ggsave(filename=paste(imgdir, env, "/traveltime-", nagents,"-robots.pdf", sep=""), width=6, height=3.2)
    traveltime.plot.w.legend <- traveltime.plot + theme(legend.position="bottom", legend.direction="horizontal", legend.box = "horizontal")
    ggsave(filename=paste(imgdir, env, "/traveltime-", nagents,"-robots-w-legend.pdf", sep=""), width=6, height=3.2)
  }
  
  # 
  # task.plot <- avgtaskprolong.vs.nagents(runs) + theme(legend.position="none")
  # ggsave(filename=paste(imgdir, env, "/prolongation.pdf", sep=""), width=6, height=3.2)
  # 
  
  # travel.time.w.legend <- traveltime.plot + theme(legend.position="bottom", legend.direction="horizontal", legend.box = "horizontal")
  # ggsave(filename=paste(imgdir, env, "/plot-with-legend.pdf", sep=""), width=10, height=6, plot=legend.grob)
  # legend <- get.legend(traveltime.plot + theme(legend.position="bottom", legend.direction="horizontal", legend.box = "horizontal"))
  # lheight <- sum(legend$heights)
  
  # 
  # legend.grob <- arrangeGrob(arrangeGrob(traveltime.plot, legend, heights=c(unit(1, "npc") - lheight, lheight), ncol=1))
  # ggsave(filename=paste(imgdir, env, "/prolong-with-legend.pdf", sep=""), width=10, height=6, plot=legend.grob)

}

imgdir <- "plots/"

make.plots(imgdir=imgdir, env="ubremen-r27")
make.plots(imgdir=imgdir, env="warehouse-r25")
make.plots(imgdir=imgdir, env="empty-hall-r25")
