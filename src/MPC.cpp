#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Timestep and dt declaration
size_t N = 7;
double dt = 0.1;

// Classroom value
const double Lf = 2.67;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;



class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    //**********************************************
    //             TUNING PARAMETERS               *
    //**********************************************
    int             cte_weight = 2800;
    int            epsi_weight = 2500;
    int        velocity_weight = 1;
    int        steering_weight = 15; 
    int           accel_weight = 20; 
    int steering_change_weight = 150; 
    int    accel_change_weight = 30; 
    int  steer_at_speed_weight = 550;
    double        target_speed = 150.0;


    // Initialize cost to 0
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (unsigned int t = 0; t < N; t++) {
      fg[0] += cte_weight * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += epsi_weight * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += velocity_weight * CppAD::pow(vars[v_start + t] - target_speed, 2);
    }


    // Minimize the use of actuators.
    for (unsigned int t = 0; t < N - 1; t++) {
      fg[0] += steering_weight * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += accel_weight * CppAD::pow(vars[a_start + t], 2);
      fg[0] += steer_at_speed_weight * CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);
    }


    // Minimize the value gap between sequential actuations.
    for (unsigned int t = 0; t < N - 2; t++) {
      fg[0] += steering_change_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += accel_change_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }


    // Set up initial constraints and increment location by 1 to
    // account for cost value at fg[0]
    fg[1 + x_start]    = vars[x_start];
    fg[1 + y_start]    = vars[y_start];
    fg[1 + psi_start]  = vars[psi_start];
    fg[1 + v_start]    = vars[v_start];
    fg[1 + cte_start]  = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Set up the remaing constraints at times
    // t=t+1 to t=t+9
    for (unsigned int t = 1; t < N; t++) {

      // The state at time t+1
      AD<double> x1    = vars[x_start + t];
      AD<double> y1    = vars[y_start + t];
      AD<double> psi1  = vars[psi_start + t];
      AD<double> v1    = vars[v_start + t];
      AD<double> cte1  = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t
      AD<double> x0    = vars[x_start + t - 1];
      AD<double> y0    = vars[y_start + t - 1];
      AD<double> psi0  = vars[psi_start + t - 1];
      AD<double> v0    = vars[v_start + t - 1];
      AD<double> cte0  = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0     = vars[a_start + t - 1];

      // Use actions from 2 timesteps ago to account
      // for system latency
      if (t > 1) {
        a0 = vars[a_start + t - 2];
        delta0 = vars[delta_start + t - 2];
      }

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
      
      
      //**********************************************************
      //                  UPDATE EQUATIONS                       *
      //**********************************************************
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);

      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta0 * dt);

      // v_[t+1] = v[t] + a[t] * dt
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta0 * dt);
    }
  }
};


//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Number of model variables based on N timesteps
  size_t n_vars = N * 6 + (N -1) * 2;
  
  // Number of constraints based on N timesteps
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set initial state variables
  double    x = state[0];
  double    y = state[1];
  double  psi = state[2];
  double    v = state[3];
  double  cte = state[4];
  double epsi = state[5];

  vars[x_start]    = x;
  vars[y_start]    = y;
  vars[psi_start]  = psi;
  vars[v_start]    = v;
  vars[cte_start]  = cte;
  vars[epsi_start] = epsi;


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set Upper and lower bounds for all variables
  for (unsigned int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -10.0e15;
    vars_upperbound[i] = 10.0e15;
  }

  // Set upper and lower limits for steering
  // The upper and lower limits of delta are set to -25 and 25
  for (unsigned int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (unsigned int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start]    = x;
  constraints_lowerbound[y_start]    = y;
  constraints_lowerbound[psi_start]  = psi;
  constraints_lowerbound[v_start]    = v;
  constraints_lowerbound[cte_start]  = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start]    = x;
  constraints_upperbound[y_start]    = y;
  constraints_upperbound[psi_start]  = psi;
  constraints_upperbound[v_start]    = v;
  constraints_upperbound[cte_start]  = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;



  vector<double> values;

  // Return Actuator Values
  values.push_back(solution.x[delta_start]);
  values.push_back(solution.x[a_start]);

  //Return x, y values
  for (unsigned int i = 0; i < N-1; i++){
    values.push_back(solution.x[x_start + i + 1]);
    values.push_back(solution.x[y_start + i + 1]);
  }

  return values;
}
