"""Setup a constrained potential dynamic game here.
"""

# TODO: cost, individual_constraints, collision_constraints

function problem_setup()
    dof = 4
    αs = [0, -pi / 2, 0, 0]
    as = [0, 0, 0.4, 0.3]
    ds = [0.4, 0.1, 0.0, 0.0]
    qs1 = [0.0, 0.0, pi / 2, -pi / 2]
    qs1 = [pi/2, 0.0, pi / 2, -pi / 2]

    r1 = Robot(dof, αs, as, ds, qs1)
    r2 = Robot(dof, αs, as, ds, qs2)

    bubbles = [(3, (0, 0, 0)), (4, (0, 0, 0))]
    collision_constraints = [(x, θ) -> norm(reduce(*, Ts[1:ii]) * [p1..., 1] - reduce(*, Ts[1:jj]) * [p2..., 1]) for (ii, p1) in bubbles, (jj, p2) in bubbles]
end


function collision_constraints(x, ii, p1, jj, p2)
    
end