# Boids Simulation in R with Border Rebound
# This script simulates flocking behavior using Craig Reynolds' Boids algorithm.
# Boids follow three rules: separation, alignment, and cohesion.
# They also bounce off the borders of the simulation window.

set.seed(42)  # For reproducibility

# --- Simulation Parameters ---
n_boids <- 50          # Number of boids
width <- 1000          # Width of simulation window
height <- 800          # Height of simulation window
max_speed <- 5         # Maximum speed of a boid
max_force <- 0.2       # Maximum steering force
vision_radius <- 50    # Radius within which boids see each other
separation_radius <- 20 # Radius within which boids avoid each other

# --- Initialize Boids ---
# Each boid has a position (x, y) and velocity (vx, vy)
boids <- data.frame(
  x = runif(n_boids, 0, width),   # Random x position
  y = runif(n_boids, 0, height),  # Random y position
  vx = runif(n_boids, -1, 1),     # Random x velocity
  vy = runif(n_boids, -1, 1)      # Random y velocity
)

# Normalize initial speed to max_speed
boids$vx <- boids$vx / sqrt(boids$vx^2 + boids$vy^2) * max_speed
boids$vy <- boids$vy / sqrt(boids$vx^2 + boids$vy^2) * max_speed

# --- Helper Functions ---

# Calculate Euclidean distance between two points
distance <- function(x1, y1, x2, y2) {
  sqrt((x1 - x2)^2 + (y1 - y2)^2)
}

# Limit a value between min and max
limit <- function(value, min_val, max_val) {
  pmin(pmax(value, min_val), max_val)
}

# --- Boids Rules ---

# Separation: Steer to avoid crowding neighbors
separation <- function(boid_index, boids) {
  steering <- c(0, 0)  # Steering force to apply
  total <- 0          # Count of neighbors within separation radius
  
  for (i in 1:nrow(boids)) {
    if (i == boid_index) next  # Skip self
    d <- distance(boids$x[boid_index], boids$y[boid_index], boids$x[i], boids$y[i])
    if (d < separation_radius) {
      # Move away from neighbors
      steering[1] <- steering[1] - (boids$x[i] - boids$x[boid_index])
      steering[2] <- steering[2] - (boids$y[i] - boids$y[boid_index])
      total <- total + 1
    }
  }
  
  if (total > 0) {
    # Average steering force
    steering <- steering / total
    # Normalize and scale to max_speed
    mag <- sqrt(steering[1]^2 + steering[2]^2)
    if (mag > 0) {
      steering <- steering / mag * max_speed
      # Subtract current velocity to get steering force
      steering <- steering - c(boids$vx[boid_index], boids$vy[boid_index])
      # Limit steering force
      steering <- limit(steering, -max_force, max_force)
    }
  }
  
  return(steering)
}

# Alignment: Steer toward the average heading of neighbors
alignment <- function(boid_index, boids) {
  steering <- c(0, 0)  # Steering force to apply
  total <- 0          # Count of neighbors within vision radius
  
  for (i in 1:nrow(boids)) {
    if (i == boid_index) next  # Skip self
    d <- distance(boids$x[boid_index], boids$y[boid_index], boids$x[i], boids$y[i])
    if (d < vision_radius) {
      # Sum velocities of neighbors
      steering[1] <- steering[1] + boids$vx[i]
      steering[2] <- steering[2] + boids$vy[i]
      total <- total + 1
    }
  }
  
  if (total > 0) {
    # Average velocity
    steering <- steering / total
    # Normalize and scale to max_speed
    mag <- sqrt(steering[1]^2 + steering[2]^2)
    if (mag > 0) {
      steering <- steering / mag * max_speed
      # Subtract current velocity to get steering force
      steering <- steering - c(boids$vx[boid_index], boids$vy[boid_index])
      # Limit steering force
      steering <- limit(steering, -max_force, max_force)
    }
  }
  
  return(steering)
}

# Cohesion: Steer toward the average position of neighbors
cohesion <- function(boid_index, boids) {
  steering <- c(0, 0)  # Steering force to apply
  total <- 0          # Count of neighbors within vision radius
  
  for (i in 1:nrow(boids)) {
    if (i == boid_index) next  # Skip self
    d <- distance(boids$x[boid_index], boids$y[boid_index], boids$x[i], boids$y[i])
    if (d < vision_radius) {
      # Sum positions of neighbors
      steering[1] <- steering[1] + boids$x[i]
      steering[2] <- steering[2] + boids$y[i]
      total <- total + 1
    }
  }
  
  if (total > 0) {
    # Average position
    steering <- steering / total
    # Steer toward average position
    steering <- steering - c(boids$x[boid_index], boids$y[boid_index])
    # Normalize and scale to max_speed
    mag <- sqrt(steering[1]^2 + steering[2]^2)
    if (mag > 0) {
      steering <- steering / mag * max_speed
      # Subtract current velocity to get steering force
      steering <- steering - c(boids$vx[boid_index], boids$vy[boid_index])
      # Limit steering force
      steering <- limit(steering, -max_force, max_force)
    }
  }
  
  return(steering)
}

# --- Main Simulation Loop ---
dev.new(width = width, height = height)  # Open a new graphics window
par(mar = rep(0, 4))  # Remove margins

for (iter in 1:200) {
  # Clear and redraw the plot
  plot(0, 0, type = "n", xlim = c(0, width), ylim = c(0, height), ann = FALSE, axes = FALSE)
  points(boids$x, boids$y, pch = 20, col = "blue")  # Draw boids
  
  for (i in 1:nrow(boids)) {
    # Apply the three rules
    sep <- separation(i, boids)
    ali <- alignment(i, boids)
    coh <- cohesion(i, boids)
    
    # Update velocity
    boids$vx[i] <- boids$vx[i] + sep[1] + ali[1] + coh[1]
    boids$vy[i] <- boids$vy[i] + sep[2] + ali[2] + coh[2]
    
    # Limit speed
    speed <- sqrt(boids$vx[i]^2 + boids$vy[i]^2)
    if (speed > max_speed) {
      boids$vx[i] <- boids$vx[i] / speed * max_speed
      boids$vy[i] <- boids$vy[i] / speed * max_speed
    }
    
    # Update position
    boids$x[i] <- boids$x[i] + boids$vx[i]
    boids$y[i] <- boids$y[i] + boids$vy[i]
    
    # Border rebound: bounce off edges
    if (boids$x[i] < 0) {
      boids$x[i] <- 0
      boids$vx[i] <- -boids$vx[i] * 0.9  # Reverse and dampen
    }
    if (boids$x[i] > width) {
      boids$x[i] <- width
      boids$vx[i] <- -boids$vx[i] * 0.9  # Reverse and dampen
    }
    if (boids$y[i] < 0) {
      boids$y[i] <- 0
      boids$vy[i] <- -boids$vy[i] * 0.9  # Reverse and dampen
    }
    if (boids$y[i] > height) {
      boids$y[i] <- height
      boids$vy[i] <- -boids$vy[i] * 0.9  # Reverse and dampen
    }
  }
  
  Sys.sleep(0.1)  # Pause for animation
}
