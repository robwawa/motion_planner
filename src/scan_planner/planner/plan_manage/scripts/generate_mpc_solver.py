#!/usr/bin/env python3
import os
import shutil
import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
OUT = os.path.join(ROOT, 'mpc_generated')
N = 12
DT = 0.08

model = AcadosModel()
model.name = 'scan_planar_mpc'
x = ca.SX.sym('x', 3)
u = ca.SX.sym('u', 3)
xdot = ca.SX.sym('xdot', 3)
c, s = ca.cos(x[2]), ca.sin(x[2])
f_expl = ca.vertcat(c*u[0] - s*u[1], s*u[0] + c*u[1], u[2])
model.x = x
model.u = u
model.p = ca.SX.sym('p', 3)
model.xdot = xdot
model.f_expl_expr = f_expl
model.f_impl_expr = xdot - f_expl
model.cost_y_expr = ca.vertcat(x, u, u - model.p)
model.cost_y_expr_e = x

ocp = AcadosOcp()
ocp.model = model
ocp.solver_options.N_horizon = N
ocp.solver_options.tf = N * DT
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.nlp_solver_type = 'SQP'
ocp.solver_options.nlp_solver_max_iter = 8
ocp.solver_options.levenberg_marquardt = 1e-3
ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
ocp.solver_options.print_level = 0
ocp.solver_options.ext_fun_compile_flags = '-O3'

ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.cost.cost_type_e = 'NONLINEAR_LS'
ocp.cost.W = np.diag([8.0, 8.0, 1.5, 0.08, 0.08, 0.08, 0.4, 0.4, 0.4])
ocp.cost.W_e = np.diag([12.0, 12.0, 2.0])
ocp.cost.yref = np.zeros(9)
ocp.cost.yref_e = np.zeros(3)

ocp.constraints.lbu = np.array([-0.8, -0.35, -1.0])
ocp.constraints.ubu = np.array([0.8, 0.35, 1.0])
ocp.constraints.idxbu = np.array([0, 1, 2])
ocp.constraints.x0 = np.zeros(3)
ocp.parameter_values = np.zeros(3)

if os.path.exists(OUT):
    shutil.rmtree(OUT)
os.makedirs(OUT, exist_ok=True)
ocp.code_gen_opts.code_export_directory = OUT
ocp.code_gen_opts.json_file = os.path.join(OUT, 'scan_planar_mpc.json')
AcadosOcpSolver.generate(ocp, json_file=ocp.code_gen_opts.json_file)
print(OUT)
