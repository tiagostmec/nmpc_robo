#include "ros/ros.h"
#include "otimizador.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <stdio.h>
#include <chrono>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   5        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;



bool solver(nmpc_robo::otimizador::Request  &req,
         nmpc_robo::otimizador::Response &res)
{

 // Inicialização das Variáveis
    int i,iter;
    double tempo;
    //time_t tstart, tend;
    acado_initializeSolver();

    // states
    for (i = 0; i < NX * (N + 1); ++i)
      acadoVariables.x[i] = 0.01;

    // controls
    for (i = 0; i < NU * N; ++i)
      acadoVariables.u[i] = 0.0;

    // reference
    for (i = 0; i < NY * N; ++i)
      acadoVariables.y[i] = 0.0;

    // reference endpoint
    for (i = 0; i < NYN; ++i)
      acadoVariables.yN[i] = 0.0;

  #if ACADO_INITIAL_STATE_FIXED
    // initial values
    for (i = 0; i < NX; ++i)
      acadoVariables.x0[i] = 0.1;
  #endif

int    acado_preparationStep();
auto start = std::chrono::high_resolution_clock::now();
// Fim da Inicialização

// Variáveis vindas do nó de controle
acadoVariables.x0[0] = req.d; // Z
acadoVariables.x0[1] = req.e; // PhiR
acadoVariables.od[0] = req.a; // roll
acadoVariables.od[1] = req.b; // pitch
acadoVariables.od[2] = req.c; // yaw
acadoVariables.od[3] = req.d; // z
acadoVariables.od[4] = req.e; // phir
acadoVariables.od[5] = req.f; // r
acadoVariables.od[6] = req.g; // H
acadoVariables.od[7] = req.h; // ux

//

// Inicio do Solver
for(iter = 0; iter < NUM_STEPS; ++iter)
        {
       acado_feedbackStep();
       acado_printControlVariables();
       acado_printDifferentialVariables();
//       acado_shiftStates(2, 0, 0);
//       acado_shiftControls(0);
       acado_preparationStep();
}
auto finish = std::chrono::high_resolution_clock::now();
auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start);
tempo = elapsed.count() * 1e-9;
// Enviar a variável de entrada do controle u para ser enviado para o código de controle
res.u = acadoVariables.u[0];
res.v = tempo;

printf("Time measured: %.5f seconds.\n", tempo);
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "otimizador");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::ServiceServer service = n.advertiseService("otimizador", solver);
  ROS_INFO("Otimizador pronto para receber argumentos");
  ros::spin();
  loop_rate.sleep();
  return 0;
}
