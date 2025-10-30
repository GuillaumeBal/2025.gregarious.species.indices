require(magrittr)
wd <- "C:/Users/gbal/Desktop/2025.gregarious.species.indices/1.boids.sim/1.ai.base.codes" %T>%
  setwd()

# Boids Simulation in R using Rcpp for high performance

# Load the Rcpp library
library(Rcpp)

# Source the C++ code
Rcpp::sourceCpp("2c.mistral.rebound.cpp")

# --- Simulation Parameters ---
n_boids <- 5000         # Number of boids (can be much larger with C++)
width <- 1000           # Width of simulation window
height <- 800           # Height of simulation window
max_speed <- 5          # Maximum speed of a boid
max_force <- 0.2        # Maximum steering force
vision_radius <- 50     # Radius within which boids see each other
separation_radius <- 20 # Radius within which boids avoid each other
sleep_time <- 0.2      # Time between frames (smaller = smoother but faster)

# --- Initialize Boids ---
# Each boid has a random position and velocity
boids <- data.frame(
  x = runif(n_boids, 0, width),
  y = runif(n_boids, 0, height),
  vx = runif(n_boids, -1, 1),
  vy = runif(n_boids, -1, 1)
)

# Normalize initial speed to max_speed
boids$vx <- boids$vx / sqrt(boids$vx^2 + boids$vy^2) * max_speed
boids$vy <- boids$vy / sqrt(boids$vx^2 + boids$vy^2) * max_speed

# --- Main Simulation Loop ---
# Open a new graphics window
#dev.new(width = width, height = height)
par(mar = rep(0, 4))  # Remove margins

# Run the simulation for 500 time steps
for (iter in 1:500) {
  # Clear and redraw the plot
  plot(0, 0, type = "n", xlim = c(0, width), ylim = c(0, height), ann = FALSE, axes = FALSE)
  # Draw all boids
  points(boids$x, boids$y, pch = 20, col = "blue")
  
  # Update boids using the C++ function
  boids <- update_boids(boids, max_speed, max_force, vision_radius, separation_radius, width, height)
  
  # Pause briefly for animation
  Sys.sleep(sleep_time)
}
