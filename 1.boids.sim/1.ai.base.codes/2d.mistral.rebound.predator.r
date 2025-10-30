library(ggplot2)
library(animation)

# Parameters
n_boids <- 50
n_predators <- 2
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

# Helper functions
limit <- function(vec, max) {
  mag <- sqrt(sum(vec^2))
  if (mag > max) {
    vec / mag * max
  } else {
    vec
  }
}

# Boids rules
separation <- function(boid, boids) {
  steering <- c(0, 0)
  total <- 0
  for (i in 1:nrow(boids)) {
    other <- boids[i, ]
    d <- sqrt((boid$x - other$x)^2 + (boid$y - other$y)^2)
    if (d > 0 && d < neighbor_radius) {
      diff <- c(boid$x - other$x, boid$y - other$y)
      diff <- diff / d
      steering <- steering + diff
      total <- total + 1
    }
  }
  if (total > 0) {
    steering <- steering / total
    steering <- limit(steering, max_speed)
    steering <- steering - c(boid$vx, boid$vy)
    steering <- limit(steering, max_force)
  }
  return(steering)
}

alignment <- function(boid, boids) {
  steering <- c(0, 0)
  total <- 0
  for (i in 1:nrow(boids)) {
    other <- boids[i, ]
    d <- sqrt((boid$x - other$x)^2 + (boid$y - other$y)^2)
    if (d > 0 && d < neighbor_radius) {
      steering <- steering + c(other$vx, other$vy)
      total <- total + 1
    }
  }
  if (total > 0) {
    steering <- steering / total
    steering <- limit(steering, max_speed)
    steering <- steering - c(boid$vx, boid$vy)
    steering <- limit(steering, max_force)
  }
  return(steering)
}

cohesion <- function(boid, boids) {
  steering <- c(0, 0)
  total <- 0
  for (i in 1:nrow(boids)) {
    other <- boids[i, ]
    d <- sqrt((boid$x - other$x)^2 + (boid$y - other$y)^2)
    if (d > 0 && d < neighbor_radius) {
      steering <- steering + c(other$x, other$y)
      total <- total + 1
    }
  }
  if (total > 0) {
    steering <- steering / total
    steering <- steering - c(boid$x, boid$y)
    steering <- limit(steering, max_speed)
    steering <- steering - c(boid$vx, boid$vy)
    steering <- limit(steering, max_force)
  }
  return(steering)
}

avoid_predators <- function(boid, predators) {
  steering <- c(0, 0)
  for (i in 1:nrow(predators)) {
    predator <- predators[i, ]
    d <- sqrt((boid$x - predator$x)^2 + (boid$y - predator$y)^2)
    if (d < predator_radius) {
      diff <- c(boid$x - predator$x, boid$y - predator$y)
      diff <- diff / d
      steering <- steering + diff
    }
  }
  if (sqrt(sum(steering^2)) > 0) {
    steering <- limit(steering, max_speed)
    steering <- steering - c(boid$vx, boid$vy)
    steering <- limit(steering, max_force)
  }
  return(steering)
}

# Update boids
update_boids <- function(boids, predators) {
  for (i in 1:nrow(boids)) {
    boid <- boids[i, ]
    sep <- separation(boid, boids)
    ali <- alignment(boid, boids)
    coh <- cohesion(boid, boids)
    pred <- avoid_predators(boid, predators)
    
    boids$vx[i] <- boids$vx[i] +
      sep[1] * separation_weight +
      ali[1] * alignment_weight +
      coh[1] * cohesion_weight +
      pred[1] * predator_avoid_weight
    boids$vy[i] <- boids$vy[i] +
      sep[2] * separation_weight +
      ali[2] * alignment_weight +
      coh[2] * cohesion_weight +
      pred[2] * predator_avoid_weight
    
    # Limit speed
    speed <- sqrt(boids$vx[i]^2 + boids$vy[i]^2)
    if (speed > max_speed) {
      boids$vx[i] <- boids$vx[i] / speed * max_speed
      boids$vy[i] <- boids$vy[i] / speed * max_speed
    }
    
    # Update position
    boids$x[i] <- boids$x[i] + boids$vx[i]
    boids$y[i] <- boids$y[i] + boids$vy[i]
    
    # Rebound off walls
    if (boids$x[i] < 0 || boids$x[i] > width) {
      boids$vx[i] <- -boids$vx[i]
      boids$x[i] <- pmax(0, pmin(width, boids$x[i]))
    }
    if (boids$y[i] < 0 || boids$y[i] > height) {
      boids$vy[i] <- -boids$vy[i]
      boids$y[i] <- pmax(0, pmin(height, boids$y[i]))
    }
  }
  return(boids)
}

# Update predators
update_predators <- function(predators, boids) {
  for (i in 1:nrow(predators)) {
    predator <- predators[i, ]
    closest_boid <- NULL
    min_dist <- Inf
    for (j in 1:nrow(boids)) {
      boid <- boids[j, ]
      d <- sqrt((predator$x - boid$x)^2 + (predator$y - boid$y)^2)
      if (d < min_dist) {
        min_dist <- d
        closest_boid <- boid
      }
    }
    if (!is.null(closest_boid)) {
      dx <- closest_boid$x - predator$x
      dy <- closest_boid$y - predator$y
      d <- sqrt(dx^2 + dy^2)
      if (d > 0) {
        predators$vx[i] <- predators$vx[i] + dx / d * 0.05
        predators$vy[i] <- predators$vy[i] + dy / d * 0.05
      }
    }
    
    # Limit speed
    speed <- sqrt(predators$vx[i]^2 + predators$vy[i]^2)
    if (speed > max_speed * 1.2) {
      predators$vx[i] <- predators$vx[i] / speed * max_speed * 1.2
      predators$vy[i] <- predators$vy[i] / speed * max_speed * 1.2
    }
    
    # Update position
    predators$x[i] <- predators$x[i] + predators$vx[i]
    predators$y[i] <- predators$y[i] + predators$vy[i]
    
    # Rebound off walls
    if (predators$x[i] < 0 || predators$x[i] > width) {
      predators$vx[i] <- -predators$vx[i]
      predators$x[i] <- pmax(0, pmin(width, predators$x[i]))
    }
    if (predators$y[i] < 0 || predators$y[i] > height) {
      predators$vy[i] <- -predators$vy[i]
      predators$y[i] <- pmax(0, pmin(height, predators$y[i]))
    }
  }
  return(predators)
}

# Animation
saveGIF({
  for (i in 1:100) {
    boids <- update_boids(boids, predators)
    predators <- update_predators(predators, boids)
    
    p <- ggplot() +
      geom_point(data = boids, aes(x = x, y = y), color = "blue", size = 2) +
      geom_point(data = predators, aes(x = x, y = y), color = "red", size = 3) +
      coord_fixed(xlim = c(0, width), ylim = c(0, height)) +
      theme_void()
    print(p)
  }
}, interval = 0.1, outdir = getwd())
