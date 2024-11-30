function collision_bubbles()
    bubbles = [
        (3, [0.0, 0.0, 0.0]),
        (3, [0.2, 0.0, 0.0]),
        (4, [0.0, 0.0, 0.0]),
        (4, [0.15, 0.0, 0.0]),
        (4, [0.3, 0.0, 0.0])
    ]
    R = 0.1
    return bubbles, R
end

function create_dynamic_game(; Δt=0.1, T=1)
    n = Int64(T / Δt) + 1

    f(x, θ) =
        let
            xf1, xf2 = blocks(θ)[3:4]
            x1, u1, x2, u2 = map(blocks, blocks(x))
            c1 = sum(norm_sqr(x1[tt] - xf1) + 0.01norm_sqr(u1[tt]) for tt in 1:n-1)
            c2 = sum(norm_sqr(x2[tt] - xf2) + 0.01norm_sqr(u2[tt]) for tt in 1:n-1)
            c1 + c2 + norm_sqr(x1[end] - xf1) + norm_sqr(x2[end] - xf2)
        end

    g(x, θ) =
        let
            x1, u1, x2, u2 = map(blocks, blocks(x))
            θ = blocks(θ)
            A = (I-diagm(1 => [I for _ in 1:(n-1)]))[1:(n-1), :]
            vcat(x1[1] - θ[1], (A * x1 + u1 * Δt)..., x2[1] - θ[2], (A * x2 + u2 * Δt)...)
        end

    umax = 1
    bubbles, R = collision_bubbles()
    h(x, θ) =
        let
            x1, u1, x2, u2 = map(blocks, blocks(x))
            # @infiltrate
            vcat(-abs.(vcat(u1...)) + umax * ones(dof * (n - 1)), -abs.(vcat(u2...)) + umax * ones(dof * (n - 1)), [
                norm_sqr(forward(r1, x1[tt], ind1, p1) - forward(r2, x2[tt], ind2, p2)) - R^2
                for tt in 1:n, (ind1, p1) in bubbles, (ind2, p2) in bubbles
            ]...)
        end

    problem = ParametricOptimizationProblem(;
        objective=f,
        equality_constraint=g,
        inequality_constraint=h,
        parameter_dimension=2(r1.dof + r2.dof),
        primal_dimension=2 * dof * (2n - 1),
        equality_dimension=2 * dof * n,
        inequality_dimension=2(n - 1) * dof + n * length(bubbles)^2
    )
end
