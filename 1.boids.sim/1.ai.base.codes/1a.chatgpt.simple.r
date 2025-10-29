library(ggplot2)

# --- Simulation parameters ---
n_boids <- 50         # number of boids
width <- 100           # world width
height <- 100          # world height
max_speed <- 2
view_radius <- 15
separation_dist <- 5
alignment_weight <- 0.05
cohesion_weight <- 0.01
separation_weight <- 0.1
steps <- 300

# --- Initialize boids ---
set.seed(123)
boids <- data.frame(
  x = runif(n_boids, 0, width),
  y = runif(n_boids, 0, height),
  vx = runif(n_boids, -1, 1),
  vy = runif(n_boids, -1, 1)
)

# --- Helper functions ---
limit_speed <- function(vx, vy, max_speed) {
  speed <- sqrt(vx^2 + vy^2)
  factor <- ifelse(speed > max_speed, max_speed / speed, 1)
  list(vx = vx * factor, vy = vy * factor)
}

update_boids <- function(boids) {
  new_boids <- boids
  
  for (i in 1:nrow(boids)) {
    # Find neighbors
    dx <- boids$x - boids$x[i]
    dy <- boids$y - boids$y[i]
    dist <- sqrt(dx^2 + dy^2)
    neighbors <- which(dist > 0 & dist < view_radius)
    
    if (length(neighbors) > 0) {
      # Compute average direction (alignment)
      avg_vx <- mean(boids$vx[neighbors])
      avg_vy <- mean(boids$vy[neighbors])
      
      # Compute center of mass (cohesion)
      center_x <- mean(boids$x[neighbors])
      center_y <- mean(boids$y[neighbors])
      
      # Compute separation
      close_neighbors <- which(dist < separation_dist & dist > 0)
      sep_x <- sum(boids$x[i] - boids$x[close_neighbors])
      sep_y <- sum(boids$y[i] - boids$y[close_neighbors])
      
      # Apply forces
      new_boids$vx[i] <- boids$vx[i] +
        alignment_weight * (avg_vx - boids$vx[i]) +
        cohesion_weight * (center_x - boids$x[i]) +
        separation_weight * sep_x
      
      new_boids$vy[i] <- boids$vy[i] +
        alignment_weight * (avg_vy - boids$vy[i]) +
        cohesion_weight * (center_y - boids$y[i]) +
        separation_weight * sep_y
    }
    
    # Limit speed
    v <- limit_speed(new_boids$vx[i], new_boids$vy[i], max_speed)
    new_boids$vx[i] <- v$vx
    new_boids$vy[i] <- v$vy
    
    # Update position
    new_boids$x[i] <- (boids$x[i] + new_boids$vx[i]) %% width
    new_boids$y[i] <- (boids$y[i] + new_boids$vy[i]) %% height
  }
  
  return(new_boids)
}

# --- Run simulation and visualize ---
for (t in 1:steps) {
  boids <- update_boids(boids)
  
  # Plot every few steps
  if (t %% 10 == 0) {
    p <- ggplot(boids, aes(x, y)) +
      geom_point(color = "steelblue", size = 2) +
      coord_fixed(ratio = 1, xlim = c(0, width), ylim = c(0, height)) +
      ggtitle(paste("Boids Simulation - Step", t)) +
      theme_minimal()
    print(p)
    Sys.sleep(0.05)
  }
}
