"""Setup a constrained potential dynamic game here.
"""

using FinalProject

function create_dynamic_game(goal; Δt=0.1, T=10)
    n = Int64(T / Δt)

    xf1, xf2 = blocks(goal)
    f(x, θ) =
        let
            x1, u1, x2, u2 = map(blocks, blocks(x))
            c1 = sum(norm_sqr(x1[tt] - xf1) + 0.01norm_sqr(u1[tt]) for tt in 1:n-1)
            c2 = sum(norm_sqr(x2[tt] - xf2) + 0.01norm_sqr(u2[tt]) for tt in 1:n-1)
            c1 + c2 + norm_sqr(x1[end] - xf1) + norm_sqr(x2[end] - xf2)
        end

    g(x, θ) =
        let
            x1, u1, x2, u2 = map(blocks, blocks(x))
            A = (I-diagm(1 => [I for _ in 1:(n-1)]))[1:(n-1), :]
            vcat((A * x1 + u1 * Δt)..., (A * x2 + u2 * Δt)...)
        end

    bubbles = [(4, (0, 0, 0))]
    R = 1
    h(x, θ) =
        let
            x1, u1, x2, u2 = map(blocks, blocks(x))
            vcat(u1..., u2..., [
                norm_sqr(forward(r1, x1[tt], ind1, p1) - forward(r2, x2[tt], ind2, p2)) - R
                for tt in 1:n, (ind1, p1) in bubbles, (ind2, p2) in bubbles
            ]...)
        end

    problem = ParametricOptimizationProblem(;
        objective=f,
        equality_constraint=g,
        inequality_constraint=h,
        parameter_dimension=1,
        primal_dimension=2 * dof * (2n - 1),
        equality_dimension=2 * dof * (n - 1),
        inequality_dimension=2(n - 1) * dof + Int64(n * length(bubbles) * (length(bubbles) + 1) / 2)
    )
end
