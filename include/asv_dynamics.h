#ifndef ASV_DYNAMICS_H
#define ASV_DYNAMICS_H

#include <array>
#include <Eigen/Dense>
#include "sea_surface_dynamics.h"
#include "geometry.h"

namespace asv_swarm
{
namespace Hydrodynamics
{
/**
 * A simple structure to contain all input data about ASV.
 * **Note:** All dimensions are with body-fixed frame of reference (and not 
 * global frame of reference).
 */
struct ASV_particulars
{
public:
  Quantity<Units::length> L; // length on load water line
  Quantity<Units::length> B; // beam of ASV at midship
  Quantity<Units::length> T; // draft of ASV at midship
  Quantity<Units::length> D; // depth of the ASV at midship
  Quantity<Units::volume> displacement; // displacement at load water line
  Geometry::Point centre_of_gravity; // Also the control point of ASV. The COG 
                                     // is measured with respect to body-fixed 
                                     // reference frame.
  Quantity<Units::length> metacentric_height; // metacentric height from keel
  Quantity<Units::length> r_roll; // roll radius of gyration
  Quantity<Units::length> r_pitch; // pitch radius of gyration
  Quantity<Units::length> r_yaw; // yaw radius of gyration
  Quantity<Units::velocity> max_speed; // maximum operational speed of the ASV.
};

struct ASV_motion_state
{
public:
  Eigen::Matrix<double, 6, 1 > position; // linear and angular displacements
  Eigen::Matrix<double, 6, 1> velocity; // linear and angular velocities
  Eigen::Matrix<double, 6, 1> acceleration; // linear and angular accelerations
};

/**
 * Class for calculation the dynamics of ASV in waves, wind and current.
 */
class ASV_dynamics
{
public:
  /**
   * Constructor. 
   * @param sea_surface reference to the sea surface simulated.
   * @param asv is the particulars of the ASV.
   * @param initial_state is the initial state of the ASV.
   */
  ASV_dynamics(Sea_surface_dynamics& sea_surface, 
               ASV_particulars asv, 
               ASV_motion_state initial_state);

  /**
   * Method to update the position and attitude of the ASV in the global 
   * coordinates for the current time step.
   */
  void set_position(Quantity<Units::time> current_time);

private:
  /**
   * Method to calculate added mass and set set mass matrix.
   */
  void set_mass_matrix();

  /**
   * Method to calculate the damping coefficient. 
   */
  void set_damping_matrix();

  /**
   * Method to calculate the hydrostatic stiffness matrix.
   */
  void set_stiffness_matrix();

  /**
   * Method to calculate the wave force and moments due to wave of unit height 
   * for a range of frequencies and heading angles.
   */
  void set_unit_wave_force_spectrum();

  /**
   * Set the wave for matrix for the current time step.
   */
  void set_wave_force_matrix();

  /**
   * Set the propeller force matrix for the current time step.
   */
  void set_propeller_force_matrix();

  /**
   * Set the water current force matrix for the current time step.
   */
  void set_current_force_matrix();

  /**
   * Set the wind force matrix for the current time step.
   */
  void set_wind_force_matrix();
  /**
   * Method to set the restoring force matrix for the current time step. 
   * Restoring force = buoyancy - weight.
   */
  void set_restoring_force_matrix();

  /**
   * Method to set the damping force matrix for the current time step.
   */
  void set_damping_force_matrix();

  /**
   * Method to get the encounter frequency for a given regular wave.
   * @param asv_speed is he speed of the ASV.
   * @param wave_frequency is the frequency of the regular wave.
   * @param wave_heading is the direction of propagation of the wave with
   * respect to direction of propagation of the ASV.
   */
  Quantity<Units::frequency> get_encounter_frequency(
      Quantity<Units::velocity> asv_speed,
      Quantity<Units::frequency> wave_frequency,
      Quantity<Units::plane_angle> wave_heading);

private:
  ASV_particulars asv; // Reference to the ASV simulated.

  Sea_surface_dynamics& sea_surface; // Reference to the sea surface simulated.
  Quantity<Units::frequency> min_encounter_frequency;
  Quantity<Units::frequency> max_encounter_frequency;
  static const int freq_count {101}; // Number of discrete frequencies
                                     // in the unit wave force spectrum. 
  static const int direction_count {361}; // Number of discrete wave 
                                          // headings in the unit wave force 
                                          // spectrum.
  static const int dof {6}; // degrees of freedom
  
  Quantity<Units::time> current_time;
  ASV_motion_state motion_state; // State of motion of the ASV for the current 
                                 // time step.
  Eigen::Matrix<double, dof, dof> M; // Mass matrix. mass + added mass
  Eigen::Matrix<double, dof, dof> C; // Damping matrix. Viscous damping coefficient 
  Eigen::Matrix<double, dof, dof> K; // Stiffness matrix.
  std::array<std::array<std::array<double, dof>, freq_count>, direction_count> 
    F_unit_wave;                         // Unit wave force spectrum.
  Eigen::Matrix<double, dof, 1> F_damping{};     // Damping force matrix
  Eigen::Matrix<double, dof, 1> F_restoring{};   // Restoring force matrix  
  Eigen::Matrix<double, dof, 1> F_wave{}; // Wave force matrix 
  Eigen::Matrix<double, dof, 1> F_wind{};        // wind force matrix
  Eigen::Matrix<double, dof, 1> F_current{};     // Water current force matrix
  Eigen::Matrix<double, dof, 1> F_propulsion{};  // Propeller thrust force matrix
};

} //namespace Hydrodynamics

} //namespace asv_swarm

#endif // ASV_DYNAMICS_H
