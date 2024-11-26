using FinalProject

using BlockArrays
using RigidBodyDynamics
using MeshCatMechanisms

function check_kinematics()
    # Using RigidBodyDynamics
    mechanism = build_mechanism()
    state = MechanismState(mechanism)

    # Check coordinates matched between the two models
    point = [1, 2, 3]
    for ii in 1:r1.dof
        p1 = transform(state, Point3D(default_frame(findbody(mechanism, "link$(ii+1)")), point...), root_frame(mechanism))
        p2 = forward(r1, ii, point)
        @assert maximum(p1.v - p2) < 0.01 "$(p1.v) != $p2"
    end

    println("Correct!")
end

function check_ik()
    point = [0.5, 0.5, 0.5]
    ee = [0.2, 0, 0]

    q = ik!(r1, ee, point)
    println(point)
    println(forward(r1, q, dof, ee))
end

function check_dynamic_game()
    goal = mortar([[0, 0, 0, 0], [0, 0, 0, 0]])
    f = create_dynamic_game(goal; Δt=.5, T=2)
end

function check_game_solution()
    goal = mortar([[1, 1, 1, 1], [2, 2, 2, 2]])
    problem = create_dynamic_game(goal; Δt=0.1, T=1)
    solve(problem)
end