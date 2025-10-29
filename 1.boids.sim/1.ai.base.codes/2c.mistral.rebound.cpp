#include <Rcpp.h>
using namespace Rcpp;

// [[Rcpp::export]]
// This function updates the positions and velocities of all boids for one time step.
// It implements the three Boids rules: separation, alignment, and cohesion.
// It also handles border rebound.
DataFrame update_boids(DataFrame boids, double max_speed, double max_force,
                       double vision_radius, double separation_radius,
                       double width, double height) {
  
  // Number of boids
  int n = boids.nrows();
  // Extract position and velocity vectors
  NumericVector x = boids["x"];
  NumericVector y = boids["y"];
  NumericVector vx = boids["vx"];
  NumericVector vy = boids["vy"];
  
  // Vectors to store updated positions and velocities
  NumericVector new_vx(n);
  NumericVector new_vy(n);
  NumericVector new_x(n);
  NumericVector new_y(n);
  
  // Loop over each boid
  for (int i = 0; i < n; i++) {
    // Initialize steering forces for the three rules
    double sep_x = 0, sep_y = 0;  // Separation
    double ali_x = 0, ali_y = 0;  // Alignment
    double coh_x = 0, coh_y = 0;  // Cohesion
    int sep_count = 0, ali_count = 0, coh_count = 0;  // Count neighbors for each rule
    
    // Loop over all other boids to calculate steering forces
    for (int j = 0; j < n; j++) {
      if (i == j) continue;  // Skip self
      double dx = x[j] - x[i];
      double dy = y[j] - y[i];
      double d = sqrt(dx*dx + dy*dy);  // Distance between boids i and j
      
      // Separation: avoid crowding neighbors
      if (d < separation_radius) {
        sep_x -= dx;
        sep_y -= dy;
        sep_count++;
      }
      // Alignment and Cohesion: only consider boids within vision radius
      if (d < vision_radius) {
        ali_x += vx[j];
        ali_y += vy[j];
        ali_count++;
        coh_x += x[j];
        coh_y += y[j];
        coh_count++;
      }
    }
    
    // --- Apply Separation ---
      if (sep_count > 0) {
        double sep_mag = sqrt(sep_x*sep_x + sep_y*sep_y);
        if (sep_mag > 0) {
          // Normalize and scale to max_speed, then subtract current velocity
          sep_x = sep_x / sep_mag * max_speed - vx[i];
          sep_y = sep_y / sep_mag * max_speed - vy[i];
          // Limit the steering force
          sep_x = std::max(-max_force, std::min(max_force, sep_x));
          sep_y = std::max(-max_force, std::min(max_force, sep_y));
        }
      }
    
    // --- Apply Alignment ---
      if (ali_count > 0) {
        ali_x = ali_x / ali_count;
        ali_y = ali_y / ali_count;
        double ali_mag = sqrt(ali_x*ali_x + ali_y*ali_y);
        if (ali_mag > 0) {
          ali_x = ali_x / ali_mag * max_speed - vx[i];
          ali_y = ali_y / ali_mag * max_speed - vy[i];
          ali_x = std::max(-max_force, std::min(max_force, ali_x));
          ali_y = std::max(-max_force, std::min(max_force, ali_y));
        }
      }
    
    // --- Apply Cohesion ---
      if (coh_count > 0) {
        coh_x = coh_x / coh_count - x[i];
        coh_y = coh_y / coh_count - y[i];
        double coh_mag = sqrt(coh_x*coh_x + coh_y*coh_y);
        if (coh_mag > 0) {
          coh_x = coh_x / coh_mag * max_speed - vx[i];
          coh_y = coh_y / coh_mag * max_speed - vy[i];
          coh_x = std::max(-max_force, std::min(max_force, coh_x));
          coh_y = std::max(-max_force, std::min(max_force, coh_y));
        }
      }
    
    // --- Update Velocity ---
      new_vx[i] = vx[i] + sep_x + ali_x + coh_x;
    new_vy[i] = vy[i] + sep_y + ali_y + coh_y;
    
    // --- Limit Speed ---
      double speed = sqrt(new_vx[i]*new_vx[i] + new_vy[i]*new_vy[i]);
    if (speed > max_speed) {
      new_vx[i] = new_vx[i] / speed * max_speed;
      new_vy[i] = new_vy[i] / speed * max_speed;
    }
    
    // --- Update Position ---
      new_x[i] = x[i] + new_vx[i];
    new_y[i] = y[i] + new_vy[i];
    
    // --- Border Rebound ---
      if (new_x[i] < 0) {
        new_x[i] = 0;
        new_vx[i] = -new_vx[i] * 0.9;  // Reverse and dampen
      }
    if (new_x[i] > width) {
      new_x[i] = width;
      new_vx[i] = -new_vx[i] * 0.9;
    }
    if (new_y[i] < 0) {
      new_y[i] = 0;
      new_vy[i] = -new_vy[i] * 0.9;
    }
    if (new_y[i] > height) {
      new_y[i] = height;
      new_vy[i] = -new_vy[i] * 0.9;
    }
  }
  
  // Return the updated boids as a DataFrame
  return DataFrame::create(
    Named("x") = new_x,
    Named("y") = new_y,
    Named("vx") = new_vx,
    Named("vy") = new_vy
  );
}
