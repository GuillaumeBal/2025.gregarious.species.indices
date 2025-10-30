rm(list = ls())

require(magrittr)
require(animation)
require(ggplot2)

wd <- "C:/Users/gbal/Desktop/2025.gregarious.species.indices/1.boids.sim/2.my.version" %T>% 
  setwd()

Rcpp::sourceCpp("1.rebound.poor.areas.cpp")

# Parameters
n_boids <- 5000
n_predators <- 10
n_areas <- 10
width <- 1000
height <- 1000
max_speed <- 20#2
max_force <- 0.05
neighbor_radius <- width / 10 #15, the bigger the less groups you get
separation_weight <- 1#0.5
alignment_weight <- 0.1
cohesion_weight <- 0.1
predator_radius <- round(width / 20) # change width exclusion zone
area_radius <- round(width / runif(n_areas ,10 ,30)) # change width exclusion zone
predator_avoid_weight <- 1.5
area_avoid_weight <- 1
pred_rel_speed <- 1.5

# Initialize boids
set.seed(42)
boids <- data.frame(
  x = runif(n_boids, 0, width),
  y = runif(n_boids, 0, height),
  vx = runif(n_boids, -1, 1),
  vy = runif(n_boids, -1, 1)
)

# Initialize predators
predators <- data.frame(
  x = runif(n_predators, 0, width),
  y = runif(n_predators, 0, height),
  vx = runif(n_predators, -0.5, 0.5),
  vy = runif(n_predators, -0.5, 0.5)
)

# Initialize areas
areas <- data.frame(
  x = runif(n_areas, 0, width),
  y = runif(n_areas, 0, height),
  vx = runif(n_areas, -0.5, 0.5),
  vy = runif(n_areas, -0.5, 0.5)
)

# Animation
saveGIF({
  for (i in 1:300) {
    boids <- update_boids_cpp(boids, predators, areas, width, height, max_speed, max_force,
                              neighbor_radius, predator_radius, area_radius,
                              separation_weight, alignment_weight,
                              cohesion_weight, predator_avoid_weight, area_avoid_weight)
    
    predators <- update_predators_cpp(predators, boids, width, height, max_speed, pred_rel_speed)
    
    p <- ggplot() +
      geom_point(data = boids, aes(x = x, y = y), color = "blue", size = .5) +
      geom_point(data = predators, aes(x = x, y = y), color = "red", size = 3) +
      geom_point(data = areas, aes(x = x, y = y), color = "black", size = 3) +
      coord_fixed(xlim = c(0, width), ylim = c(0, height)) +
      theme_void()
    print(p)
  }
}, interval = 0.1, outdir = getwd())
