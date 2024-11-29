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
as = [0, 0, 0.5, 0.4]
ds = [0.2, 0.1, 0.0, 0.0]
qs = [0.0, 0.0, 3pi / 4, -pi / 2]
ee = [0.3, 0, 0]
r1 = Robot(αs, as, ds, qs, ee)
r2x = 0.8
r2y = 0.8
r2 = Robot(αs, as, ds, qs, ee, [0 1 0 r2x; -1 0 0 r2y; 0 0 1 0; 0 0 0 1])
export r1, r2

include("parametric_optimization_problem.jl")
export ParametricOptimizationProblem, solve, total_dim

include("game.jl")
export collision_bubbles, create_dynamic_game
end