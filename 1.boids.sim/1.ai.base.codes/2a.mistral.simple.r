# Boids Simulation in R

set.seed(42) # For reproducibility

# Parameters
n_boids <- 50
width <- 1000
height <- 800
max_speed <- 5
max_force <- 0.2
vision_radius <- 50
separation_radius <- 20

# Initialize boids
boids <- data.frame(
  x = runif(n_boids, 0, width),
  y = runif(n_boids, 0, height),
  vx = runif(n_boids, -1, 1),
  vy = runif(n_boids, -1, 1)
)

# Normalize speed
boids$vx <- boids$vx / sqrt(boids$vx^2 + boids$vy^2) * max_speed
boids$vy <- boids$vy / sqrt(boids$vx^2 + boids$vy^2) * max_speed

# Helper functions
distance <- function(x1, y1, x2, y2) {
  sqrt((x1 - x2)^2 + (y1 - y2)^2)
}

limit <- function(value, min_val, max_val) {
  pmin(pmax(value, min_val), max_val)
}

# Rules
separation <- function(boid, boids) {
  steering <- c(0, 0)
  total <- 0
  
  for (i in 1:nrow(boids)) {
    d <- distance(boid$x, boid$y, boids$x[i], boids$y[i])
    if (i != boid && d < separation_radius) {
      steering[1] <- steering[1] - (boids$x[i] - boid$x)
      steering[2] <- steering[2] - (boids$y[i] - boid$y)
      total <- total + 1
    }
  }
  
  if (total > 0) {
    steering <- steering / total
    mag <- sqrt(steering[1]^2 + steering[2]^2)
    if (mag > 0) {
      steering <- steering / mag * max_speed
      steering <- steering - c(boid$vx, boid$vy)
      steering <- limit(steering, -max_force, max_force)
    }
  }
  
  return(steering)
}

alignment <- function(boid, boids) {
  steering <- c(0, 0)
  total <- 0
  
  for (i in 1:nrow(boids)) {
    d <- distance(boid$x, boid$y, boids$x[i], boids$y[i])
    if (i != boid && d < vision_radius) {
      steering[1] <- steering[1] + boids$vx[i]
      steering[2] <- steering[2] + boids$vy[i]
      total <- total + 1
    }
  }
  
  if (total > 0) {
    steering <- steering / total
    mag <- sqrt(steering[1]^2 + steering[2]^2)
    if (mag > 0) {
      steering <- steering / mag * max_speed
      steering <- steering - c(boid$vx, boid$vy)
      steering <- limit(steering, -max_force, max_force)
    }
  }
  
  return(steering)
}

cohesion <- function(boid, boids) {
  steering <- c(0, 0)
  total <- 0
  
  for (i in 1:nrow(boids)) {
    d <- distance(boid$x, boid$y, boids$x[i], boids$y[i])
    if (i != boid && d < vision_radius) {
      steering[1] <- steering[1] + boids$x[i]
      steering[2] <- steering[2] + boids$y[i]
      total <- total + 1
    }
  }
  
  if (total > 0) {
    steering <- steering / total
    steering <- steering - c(boid$x, boid$y)
    mag <- sqrt(steering[1]^2 + steering[2]^2)
    if (mag > 0) {
      steering <- steering / mag * max_speed
      steering <- steering - c(boid$vx, boid$vy)
      steering <- limit(steering, -max_force, max_force)
    }
  }
  
  return(steering)
}

# Main loop
dev.new(width = width, height = height)
par(mar = rep(0, 4))

for (iter in 1:200) {
  plot(0, 0, type = "n", xlim = c(0, width), ylim = c(0, height), ann = FALSE, axes = FALSE)
  points(boids$x, boids$y, pch = 20, col = "blue")
  
  for (i in 1:nrow(boids)) {
    sep <- separation(boids[i, ], boids)
    ali <- alignment(boids[i, ], boids)
    coh <- cohesion(boids[i, ], boids)
    
    boids$vx[i] <- boids$vx[i] + sep[1] + ali[1] + coh[1]
    boids$vy[i] <- boids$vy[i] + sep[2] + ali[2] + coh[2]
    
    speed <- sqrt(boids$vx[i]^2 + boids$vy[i]^2)
    if (speed > max_speed) {
      boids$vx[i] <- boids$vx[i] / speed * max_speed
      boids$vy[i] <- boids$vy[i] / speed * max_speed
    }
    
    boids$x[i] <- boids$x[i] + boids$vx[i]
    boids$y[i] <- boids$y[i] + boids$vy[i]
    
    # Wrap around edges
    if (boids$x[i] < 0) boids$x[i] <- width
    if (boids$x[i] > width) boids$x[i] <- 0
    if (boids$y[i] < 0) boids$y[i] <- height
    if (boids$y[i] > height) boids$y[i] <- 0
  }
  
  Sys.sleep(0.1)
}