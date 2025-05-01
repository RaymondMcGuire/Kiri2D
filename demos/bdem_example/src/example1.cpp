/*
 * File: main.cpp
 * Module: src
 * Created Date: 2024-05-02
 * Author: Modified from Xu.WANG's code
 * -----
 * Last Modified: 2024-05-02
 * -----
 * Description: BDEM simulation with configurable view and object parameters
 */

#include <kiri2d.h>

using namespace KIRI2D;

inline double clampRad(double a) {
  return a < -KIRI_PI<double>()
             ? clampRad(a + 2.0 * KIRI_PI<double>())
             : (a > KIRI_PI<double>() ? clampRad(a - 2.0 * KIRI_PI<double>())
                                      : a);
}

struct Vec {
  double x, y;

  Vec(double x = 0.0, double y = 0.0) : x(x), y(y) {}

  Vec operator+(const Vec &a) const { return Vec(x + a.x, y + a.y); }

  Vec operator-(const Vec &a) const { return Vec(x - a.x, y - a.y); }

  Vec operator*(double a) const { return Vec(x * a, y * a); }

  Vec operator/(double a) const { return Vec(x / a, y / a); }

  double len() { return std::sqrt(x * x + y * y); }

  double dot(const Vec &a) { return x * a.x + y * a.y; }
};

struct Bond {
  // j:connected particle id
  int s, j;

  // initial bond length
  double l;

  // bond initial direction
  Vec d;
  double tn, tt;

  Bond(int j, int s, double l, Vec d, double tn, double tt)
      : j(j), s(s), l(l), d(d), tn(tn), tt(tt) {}
};

struct Particle {
  double im, iI, r, q, w, wMid, T;
  Vec x, v, vMid, F;
  int c;
  std::vector<Bond> bonds;

  Particle(double im, double iI, double r, Vec x, int c)
      : im(im), iI(iI), r(r), x(x), c(c), wMid(0), w(0), q(0), v(0, 0),
        vMid(0, 0), F(0, 0), T(0) {}
};

// Boundary structure for easier management
struct Boundary {
  Vec start;    // Starting point
  Vec end;      // Ending point
  Vec velocity; // Velocity for moving boundaries
  int color_id; // Color identifier for rendering
  bool movable; // Whether this boundary can move
};

int main(int argc, char *argv[]) {
  // Log system
  KiriLog::init();

  // ====== SIMULATION CONFIGURATION ======
  // These are all the parameters you need to adjust

  // View configuration
  float window_height = 1000.f;
  float window_width = 1000.f;

  // Simulation world size (in world units)
  double world_width = 2.0;  // Increase this to make the world larger
  double world_height = 2.0; // Increase this to make the world taller

  // Viewport configuration - controls what portion of the world is visible
  double view_center_x = world_width / 2.0;
  double view_center_y = world_height / 2.0;
  double view_width = world_width;   // How much of the world width to show
  double view_height = world_height; // How much of the world height to show

  // BDEM object configuration
  double object_center_x = world_width / 2.0;  // Center X of the BDEM object
  double object_center_y = world_height / 2.0; // Center Y of the BDEM object
  double object_size = 1.5;                    // Size of the square BDEM object
  double hollow_ratio = 0.1; // Hollow center radius as ratio of object_size

  // Particle configuration
  double particle_radius = 0.02;
  double particle_density = 2710.0;

  // Physics configuration
  double kn = 1e7;                   // Normal stiffness
  double mu = 0.7;                   // Friction coefficient
  double bond_strength_ratio = 0.07; // Bond strength as ratio of kn

  // Compression configuration
  double compression_speed = 0.1;    // Units per second
  double max_compression_dist = 0.2; // Maximum compression distance
  bool oscillating = false;          // Set to true for oscillating boundaries

  // ====== END OF CONFIGURATION ======

  // Calculated parameters
  double particle_mass =
      KIRI_PI<double>() * particle_radius * particle_radius * particle_density;
  double particle_inertia =
      0.5 * particle_mass * particle_radius * particle_radius;
  double hollow_radius = object_size * hollow_ratio;

  // Calculate viewport scaling and offset
  double scale_factor =
      std::min(window_width / view_width, window_height / view_height);
  Vector2F view_offset(window_width / 2.0f - view_center_x * scale_factor,
                       window_height / 2.0f - view_center_y * scale_factor);

  // Initialize scene and renderer
  auto scene = std::make_shared<KiriScene2D<float>>((size_t)window_width,
                                                    (size_t)window_height);
  auto renderer = std::make_shared<KiriRenderer2D<float>>(scene);

  // Simulation parameters
  int numOfFrames = 2000;
  double fps = 60;

  // Initialize particles container
  std::vector<Particle> pts;

  // Create the BDEM object (square with hollow center)
  double start_x = object_center_x - object_size / 2;
  double start_y = object_center_y - object_size / 2;
  double end_x = object_center_x + object_size / 2;
  double end_y = object_center_y + object_size / 2;

  for (double x = start_x; x <= end_x; x += 2 * particle_radius) {
    for (double y = start_y; y <= end_y; y += 2 * particle_radius) {
      // Calculate distance from object center
      double dx = x - object_center_x;
      double dy = y - object_center_y;
      double distance_from_center = std::sqrt(dx * dx + dy * dy);

      // Only add particles that are outside the hollow circle
      if (distance_from_center >= hollow_radius) {
        pts.push_back(Particle(1.0 / particle_mass, 1.0 / particle_inertia,
                               particle_radius, Vec(x, y),
                               1 // Color ID for BDEM particles
                               ));
      }
    }
  }

  // Create bonds between nearby particles
  for (int i = 0; i < pts.size(); i++) {
    for (int j = 0; j < pts.size(); j++) {
      if (i != j) {
        Vec l = pts[j].x - pts[i].x;
        double dist = l.len();
        // Create bonds between particles that are close enough
        if (dist <= 2.5 * particle_radius) {
          pts[i].bonds.push_back(Bond(j, 0, dist, l / dist,
                                      bond_strength_ratio * kn,
                                      bond_strength_ratio * kn));
        }
      }
    }
  }

  // Create boundaries
  std::vector<Boundary> boundaries;

  // Add bottom boundary (movable)
  boundaries.push_back({
      Vec(0.0, 0.0),               // Start
      Vec(world_width, 0.0),       // End
      Vec(0.0, compression_speed), // Initial velocity (upward)
      0,                           // Color ID
      true                         // Movable
  });

  // Add top boundary (movable)
  boundaries.push_back({
      Vec(0.0, world_height),         // Start
      Vec(world_width, world_height), // End
      Vec(0.0, -compression_speed),   // Initial velocity (downward)
      0,                              // Color ID
      true                            // Movable
  });

  // Add left boundary (fixed)
  boundaries.push_back({
      Vec(0.0, 0.0),          // Start
      Vec(0.0, world_height), // End
      Vec(0.0, 0.0),          // No velocity
      0,                      // Color ID
      false                   // Not movable
  });

  // Add right boundary (fixed)
  boundaries.push_back({
      Vec(world_width, 0.0),          // Start
      Vec(world_width, world_height), // End
      Vec(0.0, 0.0),                  // No velocity
      0,                              // Color ID
      false                           // Not movable
  });

  // Create boundary particles
  std::vector<std::vector<int>> boundary_indices(boundaries.size());

  for (size_t b = 0; b < boundaries.size(); b++) {
    Boundary &boundary = boundaries[b];

    // Calculate direction and length
    Vec direction = boundary.end - boundary.start;
    double length = direction.len();
    Vec unit_dir = direction / length;

    // Calculate number of particles needed
    int num_particles = static_cast<int>(length / (2 * particle_radius)) + 1;

    // Create evenly spaced particles along the boundary
    for (int i = 0; i < num_particles; i++) {
      double t = i / static_cast<double>(num_particles - 1);
      Vec pos = boundary.start + direction * t;

      // Add boundary particle (immovable, so im and iI are 0)
      pts.push_back(
          Particle(0.0, 0.0, particle_radius, pos, boundary.color_id));
      boundary_indices[b].push_back(pts.size() - 1);

      // Set initial velocity for movable boundaries
      if (boundary.movable) {
        pts[pts.size() - 1].vMid = boundary.velocity;
      }
    }
  }

  // Calculate simulation sub-step parameters
  int s = static_cast<int>(
              1.0 / fps /
              std::sqrt(7.5e3 * particle_radius * particle_radius / kn)) *
          10;
  double dt = 1.0 / fps / static_cast<double>(s);

  // Compression tracking variables
  double current_compression = 0.0;
  bool compressing = true;
  double compression_step = compression_speed / fps;
  int frame_counter = 0;

  // Main simulation loop
  while (frame_counter < numOfFrames) {
    frame_counter++;

    // Handle boundary movement
    if (oscillating) {
      // Oscillating boundaries
      if (current_compression >= max_compression_dist) {
        // Start releasing
        compressing = false;
        // Reverse direction of movable boundaries
        for (size_t b = 0; b < boundaries.size(); b++) {
          if (boundaries[b].movable) {
            boundaries[b].velocity = boundaries[b].velocity * -1.0;

            for (int idx : boundary_indices[b]) {
              pts[idx].vMid = boundaries[b].velocity;
            }
          }
        }
      } else if (current_compression <= 0.0 && !compressing) {
        // Start compressing again
        compressing = true;
        // Reverse direction of movable boundaries
        for (size_t b = 0; b < boundaries.size(); b++) {
          if (boundaries[b].movable) {
            boundaries[b].velocity = boundaries[b].velocity * -1.0;

            for (int idx : boundary_indices[b]) {
              pts[idx].vMid = boundaries[b].velocity;
            }
          }
        }
      }

      // Update compression amount
      current_compression += compressing ? compression_step : -compression_step;

      // Move boundaries
      for (size_t b = 0; b < boundaries.size(); b++) {
        if (boundaries[b].movable) {
          for (int idx : boundary_indices[b]) {
            pts[idx].x = pts[idx].x + boundaries[b].velocity * (dt * s);
          }
        }
      }
    } else {
      // One-time compression
      if (compressing && current_compression < max_compression_dist) {
        // Move boundaries
        for (size_t b = 0; b < boundaries.size(); b++) {
          if (boundaries[b].movable) {
            for (int idx : boundary_indices[b]) {
              pts[idx].x = pts[idx].x + boundaries[b].velocity * (dt * s);
            }
          }
        }

        current_compression += compression_step;
      } else if (compressing) {
        // Stop compression when max is reached
        compressing = false;

        // Set velocities to zero
        for (size_t b = 0; b < boundaries.size(); b++) {
          if (boundaries[b].movable) {
            boundaries[b].velocity = Vec(0, 0);

            for (int idx : boundary_indices[b]) {
              pts[idx].vMid = Vec(0, 0);
            }
          }
        }
      }
    }

    // Physics simulation sub-steps
    for (int iter = 0; iter < s; iter++) {
      // Reset forces and update positions
      for (int i = 0; i < pts.size(); i++) {
        pts[i].F = Vec();
        pts[i].T = 0.0;

        // Update position
        pts[i].x = pts[i].x + pts[i].vMid * dt;

        // Update orientation
        pts[i].q = clampRad(pts[i].q + pts[i].wMid * dt);
      }

      // Calculate forces
      for (int i = 0; i < pts.size(); i++) {
        // Skip fixed boundary particles for force calculation
        if (pts[i].im > 1e-6) {
          // Contact forces
          for (int j = 0; j < pts.size(); j++) {
            if (i != j) {
              Vec lij = pts[i].x - pts[j].x;

              // Calculate overlap
              double delta_ij = pts[i].r + pts[j].r - lij.len();

              // Skip if no overlap
              if (delta_ij <= 1e-12) {
                continue;
              }

              Vec n = lij / lij.len();

              // Contact stiffness
              auto kt = 1.4 * std::sqrt(kn / (pts[i].im + pts[j].im));

              // Repulsive force
              auto F_r = n * kn * delta_ij;

              // Tangential force (friction)
              auto vij = (pts[j].v - pts[i].v);
              auto dot_epsilon = vij.dot(n);
              auto vt = vij - n * dot_epsilon;

              auto F_f = vt * -kt;

              // Apply Coulomb friction limit
              auto max_fs = F_r.len() * mu;
              if (F_f.len() > max_fs && vt.len() > 1e-12) {
                F_f = vt * (max_fs / vt.len());
              }

              pts[i].F = pts[i].F + F_r + F_f;
            }
          }

          // Bond forces
          for (int bidx = 0; bidx < pts[i].bonds.size(); bidx++) {
            Bond &b = pts[i].bonds[bidx];
            if (b.s != 0) {
              continue; // Skip broken bonds
            }

            Vec l = pts[b.j].x - pts[i].x;
            Vec n = l / l.len();

            // Tangential unit vector in 2D
            Vec t(-n.y, n.x);

            // Bond stretch or compression
            double dl = l.len() - b.l;

            // Angular deformation
            double qb = std::atan2(b.d.y, b.d.x) - std::atan2(n.y, n.x);
            double ti = clampRad(qb + pts[i].q);
            double tj = clampRad(qb + pts[b.j].q);

            // Stretching force
            Vec F_stretch = n * kn * dl;

            // Shear stiffness
            auto ks = kn / 3 * particle_radius * particle_radius / l.len();

            // Shear force
            Vec F_shear = t * ks * -(ti + tj);

            // Torque
            double T =
                kn / 6.0 * particle_radius * particle_radius * (tj - 3.0 * ti);

            // Check if bond breaks
            if ((dl > 0.0 && (F_stretch.len() / (2.0 * particle_radius) +
                              std::abs(kn / 2.0 * (tj - ti))) > b.tn) ||
                (F_shear.len() / (2.0 * particle_radius) > b.tt)) {
              b.s = 1; // Mark bond as broken
              continue;
            }

            pts[i].F = pts[i].F + F_stretch + F_shear;
            pts[i].T = pts[i].T + T;
          }
        }
      }

      // Update velocities
      for (int i = 0; i < pts.size(); i++) {
        if (pts[i].im > 1e-6) {
          // Calculate acceleration (without gravity)
          Vec acc = pts[i].F * pts[i].im;
          // If you want to include gravity, use this line instead:
          // Vec acc = pts[i].F * pts[i].im + Vec(0, -9.8);

          // Update velocities
          pts[i].v = pts[i].vMid + acc * 0.5 * dt;
          pts[i].vMid = pts[i].vMid + acc * dt;

          // Update angular velocities
          pts[i].w = pts[i].wMid + pts[i].T * pts[i].iI * 0.5 * dt;
          pts[i].wMid = pts[i].wMid + pts[i].T * pts[i].iI * dt;
        }
      }
    }

    // Render the simulation
    std::vector<KiriCircle2<float>> circles;
    for (int i = 0; i < pts.size(); i++) {
      Vector3F color;

      if (pts[i].c == 0)
        color = Vector3F(0.f); // Black for boundaries
      else if (pts[i].c == 1)
        color = Vector3F(1.f, 0.f, 0.f); // Red for square particles
      else if (pts[i].c == 2)
        color = Vector3F(0.f, 1.f, 0.f);
      else if (pts[i].c == 3)
        color = Vector3F(0.f, 0.f, 1.f);

      // Convert world coordinates to screen coordinates
      float screen_x = pts[i].x.x * scale_factor + view_offset.x;
      float screen_y = pts[i].x.y * scale_factor + view_offset.y;
      float screen_radius = pts[i].r * scale_factor;

      auto circle = KiriCircle2<float>(Vector2F(screen_x, screen_y), color,
                                       screen_radius);
      circles.emplace_back(circle);
    }

    scene->AddCircles(circles);
    renderer->DrawCanvas();

    cv::imshow("KIRI2D", renderer->GetCanvas());
    cv::waitKey(5);
    // Uncomment to save images
    // renderer->SaveImages2File();

    renderer->ClearCanvas();
    scene->Clear();
  }

  return 0;
}