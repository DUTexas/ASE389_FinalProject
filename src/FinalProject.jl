module FinalProject

using BlockArrays
using Infiltrator
using LinearAlgebra
using RigidBodyDynamics
using ParametricMCPs
using StaticArrays
using Symbolics

include("model.jl")
include("utils.jl")
include("parametric_optimization_problem.jl")
export ParametricOptimizationProblem, solve, total_dim

include("kinematics.jl")
export Robot, get_transformation_matrix, forward

include("game.jl")
export create_dynamic_game

dof = 4
αs = [0, -pi / 2, 0, 0]
as = [0, 0, 0.4, 0.3]
ds = [0.4, 0.1, 0.0, 0.0]
qs = [0.0, 0.0, pi / 2, -pi / 2]
r1 = Robot(αs, as, ds, qs)
r2 = Robot(αs, as, ds, qs, [0 -1 0 1; 1 0 0 1; 0 0 1 0; 0 0 0 1])
end