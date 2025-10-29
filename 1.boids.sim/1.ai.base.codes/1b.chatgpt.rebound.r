# Install and load ggplot2 if not already installed
if(!require(ggplot2)) install.packages("ggplot2")
library(ggplot2)

# =============================
# SIMULATION PARAMETERS
# =============================

n_boids <- 50         # Number of boids (birds)
width <- 100          # Width of the 2D world
height <- 100         # Height of the 2D world
max_speed <- 2        # Maximum allowed speed per boid

view_radius <- 15     # Distance to consider other boids as "neighbors"
separation_dist <- 5  # Distance under which boids try to separate

# Rule weights (you can tweak these to change behavior)
alignment_weight <- 0.05   # How much boids try to align velocities
cohesion_weight <- 0.01    # How much boids try to move toward center of neighbors
separation_weight <- 0.1   # How much boids try to avoid crowding

steps <- 300          # Number of simulation steps

# =============================
# INITIALIZE BOIDS
# =============================

set.seed(123)  # For reproducibility
boids <- data.frame(
  x = runif(n_boids, 0, width),   # random x position
  y = runif(n_boids, 0, height),  # random y position
  vx = runif(n_boids, -1, 1),     # random velocity x
  vy = runif(n_boids, -1, 1)      # random velocity y
)

# =============================
# HELPER FUNCTION: Limit speed
# =============================

limit_speed <- function(vx, vy, max_speed) {
  # Compute current speed magnitude
  speed <- sqrt(vx^2 + vy^2)
  
  # If faster than allowed, scale down the velocity
  factor <- ifelse(speed > max_speed, max_speed / speed, 1)
  
  list(vx = vx * factor, vy = vy * factor)
}

# =============================
# MAIN FUNCTION: Update boids
# =============================

update_boids <- function(boids) {
  # Create a copy to store updated values
  new_boids <- boids
  
  # Loop through every boid
  for (i in 1:nrow(boids)) {
    
    # Compute distance from this boid to all others
    dx <- boids$x - boids$x[i]
    dy <- boids$y - boids$y[i]
    dist <- sqrt(dx^2 + dy^2)
    
    # Identify which boids are within the view radius
    neighbors <- which(dist > 0 & dist < view_radius)
    
    # If there are neighbors, apply the 3 rules
    if (length(neighbors) > 0) {
      
      # 1️⃣ Alignment: Match velocity with neighbors
      avg_vx <- mean(boids$vx[neighbors])
      avg_vy <- mean(boids$vy[neighbors])
      
      # 2️⃣ Cohesion: Move toward the center of neighbors
      center_x <- mean(boids$x[neighbors])
      center_y <- mean(boids$y[neighbors])
      
      # 3️⃣ Separation: Avoid getting too close
      close_neighbors <- which(dist < separation_dist & dist > 0)
      sep_x <- sum(boids$x[i] - boids$x[close_neighbors])
      sep_y <- sum(boids$y[i] - boids$y[close_neighbors])
      
      # Combine all rules to update velocity
      new_boids$vx[i] <- boids$vx[i] +
        alignment_weight * (avg_vx - boids$vx[i]) +   # align with neighbors
        cohesion_weight * (center_x - boids$x[i]) +   # move toward group center
        separation_weight * sep_x                     # move away if too close
      
      new_boids$vy[i] <- boids$vy[i] +
        alignment_weight * (avg_vy - boids$vy[i]) +
        cohesion_weight * (center_y - boids$y[i]) +
        separation_weight * sep_y
    }
    
    # Limit the boid's speed to the max allowed
    v <- limit_speed(new_boids$vx[i], new_boids$vy[i], max_speed)
    new_boids$vx[i] <- v$vx
    new_boids$vy[i] <- v$vy
    
    # Update position based on velocity
    new_boids$x[i] <- boids$x[i] + new_boids$vx[i]
    new_boids$y[i] <- boids$y[i] + new_boids$vy[i]
    
    # =============================
    # BORDER REBOUND (walls)
    # =============================
    
    # If the boid goes past the left wall:
    if (new_boids$x[i] < 0) {
      new_boids$x[i] <- 0             # stick to wall
      new_boids$vx[i] <- -new_boids$vx[i]  # reverse direction (bounce)
    }
    
    # If the boid goes past the right wall:
    if (new_boids$x[i] > width) {
      new_boids$x[i] <- width
      new_boids$vx[i] <- -new_boids$vx[i]
    }
    
    # If the boid goes past the bottom wall:
    if (new_boids$y[i] < 0) {
      new_boids$y[i] <- 0
      new_boids$vy[i] <- -new_boids$vy[i]
    }
    
    # If the boid goes past the top wall:
    if (new_boids$y[i] > height) {
      new_boids$y[i] <- height
      new_boids$vy[i] <- -new_boids$vy[i]
    }
  }
  
  return(new_boids)
}

# =============================
# RUN THE SIMULATION
# =============================

for (t in 1:steps) {
  # Update all boids each time step
  boids <- update_boids(boids)
  
  # Plot every few frames for smoother animation
  if (t %% 10 == 0) {
    p <- ggplot(boids, aes(x, y)) +
      geom_point(color = "steelblue", size = 2) +
      coord_fixed(ratio = 1, xlim = c(0, width), ylim = c(0, height)) +
      ggtitle(paste("Boids Simulation with Border Rebound - Step", t)) +
      theme_minimal()
    
    print(p)
    Sys.sleep(0.05)  # pause to see movement
  }
}
