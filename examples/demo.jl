using FinalProject
using BlockArrays
using RigidBodyDynamics
using MeshCat
using MeshCatMechanisms
using GeometryBasics
using StaticArrays


function show_robot()
    mechanism = build_mechanism("src/urdf/4dof_2.urdf")
    mvis = MechanismVisualizer(mechanism, Skeleton(inertias=false))
    open(mvis)
    for body in bodies(mechanism)
        setelement!(mvis, default_frame(body))
    end

    point = [0.8, 0, 0.1]
    ee = [0.2, 0, 0]

    state = MechanismState(mechanism)
    q = configuration(state)
    qf = ik!(r1, ee, point)
    for ii in 1:4
        set_configuration!(state, findjoint(mechanism, "j$ii"), qf[ii])
    end

    set_configuration!(state, q)
    set_configuration!(mvis, configuration(state))
    setelement!(mvis, root_frame(mechanism), HyperSphere(Point(point...), 0.1))
    println(configuration(state))

    return mechanism
end

function play_game(obj1::Vector{<:Real}=[0.5, 0, 0.1], obj2::Vector{<:Real}=[1.2, 0.3, 0.1])
    mechanism = build_mechanism("src/urdf/4dof_2.urdf")
    mvis = MechanismVisualizer(mechanism, Skeleton(inertias=false))
    open(mvis)

    dof = r1.dof
    Δt = 0.5
    T = 1
    n = Int64(T / Δt) + 1
    goal = mortar([ik!(r1, r1.ee, obj1), ik!(r2, r2.ee, obj2)])
    println(goal)
    problem = create_dynamic_game(goal; Δt=Δt, T=T)
    (; primals, variables, status, info) = solve(problem)
    x = BlockArray(primals, dof * [n, n - 1, n, n - 1])
    qs = [[x[Block(1)][4ii-3:4ii] x[Block(3)][4ii-3:4ii]]'[:] for ii in 1:n]
    ts = Vector(0:Δt:T)
    MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=0.1)
end

function main()

end