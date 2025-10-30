// Include necessary headers for Rcpp and math functions
#include <Rcpp.h>  // For R/C++ interface
#include <cmath>   // For math functions like sqrt, pow
#include <cstdlib> // for rand() and RAND_MAX

// Use the Rcpp namespace to avoid prefixing everything with Rcpp::
using namespace Rcpp;

// This function updates the positions and velocities of all boids.
// It applies the three Boids rules (separation, alignment, cohesion)
// and predator avoidance, then updates positions and handles wall rebound.
// [[Rcpp::export]]
DataFrame update_boids_cpp(
    DataFrame boids,          // DataFrame of boid positions and velocities
    DataFrame predators,      // DataFrame of predator positions and velocities
    DataFrame areas,      // DataFrame of area positions and velocities
    double width,             // Width of the simulation area
    double height,            // Height of the simulation area
    double max_speed,         // Maximum speed for boids
    double max_force,         // Maximum steering force
    double neighbor_radius,   // Radius within which boids interact
    double predator_radius,   // Radius within which boids avoid predators
    NumericVector area_radius,   // Radius within which boids avoid poor areas
    double separation_weight, // Weight for separation rule
    double alignment_weight,  // Weight for alignment rule
    double cohesion_weight,   // Weight for cohesion rule
    double predator_avoid_weight, // Weight for predator avoidance
    double area_avoid_weight // Weight for area avoidance
) {
  
  srand(time(NULL)); // seed with current time
  
  // --- 1. Extract Data from R DataFrames ---
  // Get the number of boids, predators and areas
  int n_boids = boids.nrows();
  int n_predators = predators.nrows();
  int n_areas = areas.nrows();
  
  // Extract boid positions and velocities as Rcpp NumericVectors
  NumericVector x = boids["x"];  // x-coordinates of boids
  NumericVector y = boids["y"];  // y-coordinates of boids
  NumericVector vx = boids["vx"]; // x-velocities of boids
  NumericVector vy = boids["vy"]; // y-velocities of boids
  
  // Extract predator positions
  NumericVector px = predators["x"]; // x-coordinates of predators
  NumericVector py = predators["y"]; // y-coordinates of predators
  
  // Extract areas positions
  NumericVector ax = areas["x"]; // x-coordinates of areas
  NumericVector ay = areas["y"]; // y-coordinates of areas
  
  // --- 2. Helper Function: Limit Vector Magnitude ---
  // This lambda function limits the magnitude of a 2D vector to max_speed
  auto limit = [max_speed](NumericVector vec) {
    double mag = sqrt(pow(vec[0], 2) + pow(vec[1], 2)); // Calculate magnitude
    if (mag > max_speed) { // If magnitude exceeds max_speed, scale it down
      vec[0] = vec[0] / mag * max_speed;
      vec[1] = vec[1] / mag * max_speed;
    }
    return vec;
  };
  
  // --- 3. Loop Over Each Boid ---
  for (int i = 0; i < n_boids; i++) {
    // Initialize steering vectors for each rule
    NumericVector sep(2), ali(2), coh(2), pred(2), area(2); // sep=separation, ali=alignment, coh=cohesion, pred=predator avoidance, area = area_avoidance
    sep[0] = sep[1] = ali[0] = ali[1] = coh[0] = coh[1] = pred[0] = pred[1]= area[0] = area[1] = 0.0; // Initialize to zero
    int sep_total = 0, ali_total = 0, coh_total = 0; // Counters for averaging
    
    // --- 4. Calculate Separation, Alignment, Cohesion ---
    // For each other boid, calculate the three Boids rules
    for (int j = 0; j < n_boids; j++) {
      if (i == j) continue; // Skip self (a boid doesn't interact with itself)
      
      // Calculate distance between boid i and boid j
      double dx = x[i] - x[j];
      double dy = y[i] - y[j];
      double d = sqrt(dx*dx + dy*dy);
      
      // If within neighbor_radius, apply rules
      if (d < neighbor_radius) {
        // --- Separation: Steer to avoid crowding ---
        // The closer the boid, the stronger the repulsion
        sep[0] += dx / d; // Add x-component of separation vector
        sep[1] += dy / d; // Add y-component of separation vector
        sep_total++;       // Increment counter for averaging
        
        // --- Alignment: Steer toward average heading of neighbors ---
        ali[0] += vx[j];  // Add x-velocity of neighbor
        ali[1] += vy[j];  // Add y-velocity of neighbor
        ali_total++;       // Increment counter for averaging
        
        // --- Cohesion: Steer toward average position of neighbors ---
        coh[0] += x[j];   // Add x-position of neighbor
        coh[1] += y[j];   // Add y-position of neighbor
        coh_total++;      // Increment counter for averaging
      }
    }
    
    // --- 5. Calculate Predator Avoidance ---
    // For each predator, steer away if too close
    for (int k = 0; k < n_predators; k++) {
      // Calculate distance between boid i and predator k
      double dx = x[i] - px[k];
      double dy = y[i] - py[k];
      double d = sqrt(dx*dx + dy*dy);
      
      // If within predator_radius, add repulsion vector
      if (d < predator_radius) {
        pred[0] += dx / d; // Add x-component of avoidance vector
        pred[1] += dy / d; // Add y-component of avoidance vector
      }
    }
    
    // --- 5. Calculate area Avoidance ---
    // For each area, steer away if too close
    for (int k = 0; k < n_areas; k++) {
      // Calculate distance between boid i and predator k
      double dx = x[i] - ax[k];
      double dy = y[i] - ay[k];
      double d = sqrt(dx*dx + dy*dy);
      
      // If within predator_radius, add repulsion vector
      if (d < area_radius[k]) {
        area[0] += dx / d; // Add x-component of avoidance vector
        area[1] += dy / d; // Add y-component of avoidance vector
      }
    }
    
    // --- 6. Apply Weights and Update Velocity ---
    // Normalize and apply weights to each steering vector
    
    // --- Separation ---
    if (sep_total > 0) {
      sep[0] /= sep_total; // Average x-component
      sep[1] /= sep_total; // Average y-component
      sep = limit(sep);    // Limit magnitude
      sep[0] -= vx[i];    // Subtract current velocity (steering = desired - current)
      sep[1] -= vy[i];
      sep = limit(sep);    // Limit steering force
    }
    
    // --- Alignment ---
    if (ali_total > 0) {
      ali[0] /= ali_total; // Average x-component
      ali[1] /= ali_total; // Average y-component
      ali = limit(ali);    // Limit magnitude
      ali[0] -= vx[i];    // Subtract current velocity
      ali[1] -= vy[i];
      ali = limit(ali);    // Limit steering force
    }
    
    // --- Cohesion ---
    if (coh_total > 0) {
      coh[0] /= coh_total; // Average x-position
      coh[1] /= coh_total; // Average y-position
      coh[0] -= x[i];     // Subtract current position (steering toward average)
      coh[1] -= y[i];
      coh = limit(coh);    // Limit magnitude
      coh[0] -= vx[i];    // Subtract current velocity
      coh[1] -= vy[i];
      coh = limit(coh);    // Limit steering force
    }
    
    // --- Predator Avoidance ---
    if (sqrt(pow(pred[0], 2) + pow(pred[1], 2)) > 0) {
      pred = limit(pred);  // Limit magnitude
      pred[0] -= vx[i];   // Subtract current velocity
      pred[1] -= vy[i];
      pred = limit(pred);  // Limit steering force
    }
    
    // --- areas Avoidance ---
    if (sqrt(pow(area[0], 2) + pow(area[1], 2)) > 0) {
      area = limit(area);  // Limit magnitude
      area[0] -= vx[i];   // Subtract current velocity
      area[1] -= vy[i];
      area = limit(area);  // Limit steering force
    }
    
    // --- Update Velocity with Weighted Steering ---
    // Apply weights and update velocity
    vx[i] += sep[0] * separation_weight +
      ali[0] * alignment_weight +
      coh[0] * cohesion_weight +
      pred[0] * predator_avoid_weight +
      area[0] * area_avoid_weight ;
    vy[i] += sep[1] * separation_weight +
      ali[1] * alignment_weight +
      coh[1] * cohesion_weight +
      pred[1] * predator_avoid_weight +
      area[1] *area_avoid_weight;
    
    // --- 7. Limit Speed ---
    // Ensure boid does not exceed max_speed
    double speed = sqrt(vx[i]*vx[i] + vy[i]*vy[i]);
    if (speed > max_speed) {
      vx[i] = vx[i] / speed * max_speed;
      vy[i] = vy[i] / speed * max_speed;
    }
    
    // add small perturbation
    vx[i] += (rand() / (double)RAND_MAX - 0.5) * 0.1;
    vy[i] += (rand() / (double)RAND_MAX - 0.5) * 0.1;
    
    // --- 8. Update Position ---
    // Move boid according to its velocity
    x[i] += vx[i];
    y[i] += vy[i];
    
    // --- 9. Rebound Off Walls ---
    // If boid hits a wall, reverse its velocity and clamp position
    if (x[i] < 0 || x[i] > width) {
      vx[i] = -vx[i]; // Reverse x-velocity
      x[i] = std::max(0.0, std::min(width, x[i])); // Clamp x-position
    }
    if (y[i] < 0 || y[i] > height) {
      vy[i] = -vy[i]; // Reverse y-velocity
      y[i] = std::max(0.0, std::min(height, y[i])); // Clamp y-position
    }
  }
  
  // --- 10. Return Updated Boids DataFrame ---
  return DataFrame::create(
    Named("x") = x,  // Updated x-coordinates
    Named("y") = y,  // Updated y-coordinates
    Named("vx") = vx, // Updated x-velocities
    Named("vy") = vy  // Updated y-velocities
  );
}

// This function updates the positions and velocities of all predators.
// Predators chase the nearest boid and rebound off walls.
// [[Rcpp::export]]
DataFrame update_predators_cpp(
    DataFrame predators,  // DataFrame of predator positions and velocities
    DataFrame boids,      // DataFrame of boid positions and velocities
    double width,         // Width of the simulation area
    double height,        // Height of the simulation area
    double max_speed,      // Maximum speed for predators
    double pred_rel_speed // pred relative speed compared to boids
) {
  
  // --- 1. Extract Data from R DataFrames ---
  int n_predators = predators.nrows();
  int n_boids = boids.nrows();
  
  // Extract predator and boid data
  NumericVector px = predators["x"];
  NumericVector py = predators["y"];
  NumericVector pvx = predators["vx"];
  NumericVector pvy = predators["vy"];
  NumericVector bx = boids["x"];
  NumericVector by = boids["y"];
  
  // --- 2. Loop Over Each Predator ---
  for (int i = 0; i < n_predators; i++) {
    double closest_dist = INFINITY; // Initialize to a large value
    int closest_boid = -1;           // Index of closest boid
    
    // --- 3. Find Closest Boid ---
    for (int j = 0; j < n_boids; j++) {
      double dx = bx[j] - px[i];
      double dy = by[j] - py[i];
      double d = sqrt(dx*dx + dy*dy);
      if (d < closest_dist) {
        closest_dist = d;
        closest_boid = j;
      }
    }
    
    // --- 4. Steer Toward Closest Boid ---
    if (closest_boid != -1) {
      double dx = bx[closest_boid] - px[i];
      double dy = by[closest_boid] - py[i];
      double d = sqrt(dx*dx + dy*dy);
      if (d > 0) { // Avoid division by zero
        // Steer toward the closest boid
        pvx[i] += dx / d * 0.05; // Small steering force
        pvy[i] += dy / d * 0.05;
      }
    }
    
    // --- 5. Limit Speed ---
    double speed = sqrt(pvx[i]*pvx[i] + pvy[i]*pvy[i]);
    if (speed > (max_speed * pred_rel_speed)) { // Predators can move slightly faster
      pvx[i] = pvx[i] / speed * max_speed * pred_rel_speed;
      pvy[i] = pvy[i] / speed * max_speed * pred_rel_speed;
    }
    
    // --- 6. Update Position ---
    px[i] += pvx[i];
    py[i] += pvy[i];
    
    // --- 7. Rebound Off Walls ---
    if (px[i] < 0 || px[i] > width) {
      pvx[i] = -pvx[i];
      px[i] = std::max(0.0, std::min(width, px[i]));
    }
    if (py[i] < 0 || py[i] > height) {
      pvy[i] = -pvy[i];
      py[i] = std::max(0.0, std::min(height, py[i]));
    }
  }
  
  // --- 8. Return Updated Predators DataFrame ---
  return DataFrame::create(
    Named("x") = px,
    Named("y") = py,
    Named("vx") = pvx,
    Named("vy") = pvy
  );
}
