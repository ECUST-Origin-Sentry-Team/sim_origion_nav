/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_omnidirectional_robot_dynamic_H_
#define ACADOS_SOLVER_omnidirectional_robot_dynamic_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NX     6
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NZ     0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NU     3
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NP     0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NP_GLOBAL     0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NBX    3
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NBX0   6
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NBU    3
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSBX   0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSBU   0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSH    0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSH0   0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSG    0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSPHI  0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSHN   0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSGN   0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSPHIN 0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSPHI0 0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSBXN  0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NS     0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NS0    0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NSN    0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NG     0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NBXN   0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NGN    0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NY0    9
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NY     9
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NYN    6
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_N      40
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NH     0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NHN    0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NH0    0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NPHI0  0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NPHI   0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NPHIN  0
#define OMNIDIRECTIONAL_ROBOT_DYNAMIC_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct omnidirectional_robot_dynamic_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;




    // cost

    external_function_external_param_casadi *cost_y_fun;
    external_function_external_param_casadi *cost_y_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_0_fun;
    external_function_external_param_casadi cost_y_0_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_e_fun;
    external_function_external_param_casadi cost_y_e_fun_jac_ut_xt;


    // constraints







} omnidirectional_robot_dynamic_solver_capsule;

ACADOS_SYMBOL_EXPORT omnidirectional_robot_dynamic_solver_capsule * omnidirectional_robot_dynamic_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_free_capsule(omnidirectional_robot_dynamic_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_create(omnidirectional_robot_dynamic_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_reset(omnidirectional_robot_dynamic_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of omnidirectional_robot_dynamic_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_create_with_discretization(omnidirectional_robot_dynamic_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_update_time_steps(omnidirectional_robot_dynamic_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_update_qp_solver_cond_N(omnidirectional_robot_dynamic_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_update_params(omnidirectional_robot_dynamic_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_update_params_sparse(omnidirectional_robot_dynamic_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_set_p_global_and_precompute_dependencies(omnidirectional_robot_dynamic_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_solve(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_setup_qp_matrices_and_factorize(omnidirectional_robot_dynamic_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_free(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void omnidirectional_robot_dynamic_acados_print_stats(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int omnidirectional_robot_dynamic_acados_custom_update(omnidirectional_robot_dynamic_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *omnidirectional_robot_dynamic_acados_get_nlp_in(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *omnidirectional_robot_dynamic_acados_get_nlp_out(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *omnidirectional_robot_dynamic_acados_get_sens_out(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *omnidirectional_robot_dynamic_acados_get_nlp_solver(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *omnidirectional_robot_dynamic_acados_get_nlp_config(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *omnidirectional_robot_dynamic_acados_get_nlp_opts(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *omnidirectional_robot_dynamic_acados_get_nlp_dims(omnidirectional_robot_dynamic_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *omnidirectional_robot_dynamic_acados_get_nlp_plan(omnidirectional_robot_dynamic_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_omnidirectional_robot_dynamic_H_
