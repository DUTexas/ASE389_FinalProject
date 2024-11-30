"Generic description of a constrained optimization problem."
struct ParametricOptimizationProblem{T1,T2,T3,T4}
    "Objective function"
    objective::T1
    "Equality constraint"
    equality_constraint::T2
    "Inequality constraint"
    inequality_constraint::T3

    "Dimension of parameter vector"
    parameter_dimension::T4
    "Dimension of primal variable"
    primal_dimension::T4
    "Dimension of equality constraint"
    equality_dimension::T4
    "Dimension of inequality constraint"
    inequality_dimension::T4

    "Corresponding parametric MCP"
    parametric_mcp::ParametricMCP
end

function ParametricOptimizationProblem(;
    objective,
    equality_constraint,
    inequality_constraint,
    parameter_dimension=1,
    primal_dimension,
    equality_dimension,
    inequality_dimension,
)
    @assert !isnothing(equality_constraint)
    @assert !isnothing(inequality_constraint)

    total_dimension = primal_dimension + equality_dimension + inequality_dimension

    # Define symbolic variables for this MCP.
    z̃ = Symbolics.scalarize(only(@variables(z̃[1:total_dimension])))
    z = BlockArray(z̃, [primal_dimension, equality_dimension, inequality_dimension])

    n = Int64(((primal_dimension / dof / 2) + 1) / 2)
    x = BlockArray(z[Block(1)], dof * [n, n - 1, n, n - 1])
    x = mortar(map(block -> BlockArray(block, dof * ones(Int, Int(length(block) / dof))), blocks(x)))
    λ = z[Block(2)]
    μ = z[Block(3)]

    # Define a symbolic variable for the parameters.
    @variables θ̃[1:(parameter_dimension)]
    θ = BlockArray(Symbolics.scalarize(θ̃), fill(Int64(parameter_dimension / 4), 4))

    # Build symbolic expressions for objective and constraints.
    f = objective(x, θ)
    g = equality_constraint(x, θ)
    h = inequality_constraint(x, θ)

    # Build Lagrangian using f, g, h.
    L = f - λ' * g - μ' * h

    # Build F = [∇ₓL, g, h]'.
    F = vcat(Symbolics.gradient(L, x), g, h)

    # Set lower and upper bounds for z.
    z̲ = vcat(-Inf * ones(size(x)), -Inf * ones(size(λ)), zeros(size(μ)))
    z̅ = vcat(Inf * ones(size(x)), Inf * ones(size(λ)), Inf * ones(size(μ)))

    # Build parametric MCP.
    parametric_mcp = ParametricMCP(
        F, z̃, Vector(θ), z̲, z̅; 
        compute_sensitivities=false, 
        backend_options=(; parallel=Symbolics.MultithreadedForm())
    )

    ParametricOptimizationProblem(
        objective,
        equality_constraint,
        inequality_constraint,
        parameter_dimension,
        primal_dimension,
        equality_dimension,
        inequality_dimension,
        parametric_mcp,
    )
end

function total_dim(problem::ParametricOptimizationProblem)
    problem.primal_dimension + problem.equality_dimension + problem.inequality_dimension
end

"Solve a constrained parametric optimization problem."
function solve(
    problem::ParametricOptimizationProblem,
    parameter_value=zeros(problem.parameter_dimension);
    initial_guess=nothing,
    verbose=false,
)
    z0 = if !isnothing(initial_guess)
        initial_guess
    else
        zeros(total_dim(problem))
    end

    z, status, info = ParametricMCPs.solve(
        problem.parametric_mcp,
        parameter_value; # [state1, state2, goal1, goal2]
        initial_guess=z0,
        verbose,
        cumulative_iteration_limit=100000,
        proximal_perturbation=1e-2,
        use_basics=true,
        use_start=true,
    )

    primals = z[1:(problem.primal_dimension)]

    (; primals, variables=z, status, info)
end