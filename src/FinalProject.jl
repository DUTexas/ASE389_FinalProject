module FinalProject

using BlockArrays
using Infiltrator
using LinearAlgebra: diagm, I, norm_sqr
using RigidBodyDynamics
using ParametricMCPs
using StaticArrays
using Symbolics

include("kinematics.jl")
export Robot, get_transformation_matrix, forward, ik!, build_mechanism

dof = 4
αs = [0, -pi / 2, 0, 0]
as = [0, 0, 0.4, 0.3]
ds = [0.4, 0.1, 0.0, 0.0]
qs = [0.0, 0.0, pi / 2, -pi / 2]
ee = [0.2, 0, 0]
r1 = Robot(αs, as, ds, qs, ee)
r2 = Robot(αs, as, ds, qs, ee, [0 1 0 1; -1 0 0 1; 0 0 1 0; 0 0 0 1])
export r1, r2

include("parametric_optimization_problem.jl")
export ParametricOptimizationProblem, solve, total_dim

include("game.jl")
export create_dynamic_game
end