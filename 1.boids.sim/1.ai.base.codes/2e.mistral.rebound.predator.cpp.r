require(magrittr)

"C:/Users/gbal/Desktop/2025.gregarious.species.indices/1.boids.sim/1.ai.base.codes" %T>% 
  setwd()

Rcpp::sourceCpp("2e.mistral.rebound.predator.cpp")

# Parameters
n_boids <- 500
n_predators <- 10
width <- 100
height <- 100
max_speed <- 2
max_force <- 0.05
neighbor_radius <- 15
predator_radius <- 30
separation_weight <- 0.5
alignment_weight <- 0.1
cohesion_weight <- 0.1
predator_avoid_weight <- 1.5

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

# Animation
saveGIF({
  for (i in 1:100) {
    boids <- update_boids_cpp(boids, predators, width, height, max_speed, max_force,
                              neighbor_radius, predator_radius,
                              separation_weight, alignment_weight,
                              cohesion_weight, predator_avoid_weight)
    predators <- update_predators_cpp(predators, boids, width, height, max_speed)
    
    p <- ggplot() +
      geom_point(data = boids, aes(x = x, y = y), color = "blue", size = 2) +
      geom_point(data = predators, aes(x = x, y = y), color = "red", size = 3) +
      coord_fixed(xlim = c(0, width), ylim = c(0, height)) +
      theme_void()
    print(p)
  }
}, interval = 0.1, outdir = getwd())
