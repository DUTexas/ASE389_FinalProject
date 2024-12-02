using FinalProject
using BlockArrays
using ColorTypes
using Infiltrator
using RigidBodyDynamics
using MeshCat
using MeshCatMechanisms
using GeometryBasics
using StaticArrays
using Random: seed!

function visualize_robot(; bubbles=true, frame=false)
    # Create robotic arms but doesn't display them
    mechanism = build_mechanism("src/urdf/4dof_2.urdf")
    mvis = MechanismVisualizer(mechanism, Skeleton(inertias=false))

    # Show local reference frames
    if frame
        for body in bodies(mechanism)
            setelement!(mvis, default_frame(body))
        end
    end

    # Show collision bubbles
    if bubbles
        bubbles, R = collision_bubbles()
        material = MeshCat.MeshPhongMaterial(color=RGBA(0.0, 0.0, 1.0, 0.3))
        for (ind, p) in bubbles
            link = findbody(mechanism, "link$(ind+1)")
            setelement!(mvis, default_frame(link), HyperSphere(Point(p...), R), material, "r1($ind, $p)")
            link = findbody(mechanism, "link$(ind+1)_2")
            setelement!(mvis, default_frame(link), HyperSphere(Point(p...), R), material, "r2($ind, $p)")
        end
    end

    return mvis, mechanism
end

function show_robot()
    # Shows robot with collision spheres visualized
    mvis, mechanism = visualize_robot(frame=true)
    open(mvis)

    # Desired points
    point1 = [0.5, 0.1, 0.05]
    point2 = [0.8, -0.2, 0.05]

    # Show desired point
    material = MeshCat.MeshPhongMaterial(color=RGBA(0.0, 1.0, 0.0, 1))
    setelement!(mvis, root_frame(mechanism), HyperSphere(Point(point1...), 0.07), material, "s1")
    setelement!(mvis, root_frame(mechanism), HyperSphere(Point(point2...), 0.07), material, "s2")

    state = MechanismState(mechanism)
    q = configuration(state)

    # Calculate joint configurations to reach desired points
    qf1 = ik!(r1, point1)
    qf2 = ik!(r2, point2)
    for ii in 1:4
        set_configuration!(state, findjoint(mechanism, "j$(ii)"), qf1[ii])
        set_configuration!(state, findjoint(mechanism, "j$(ii)_2"), qf2[ii])
    end

    # Set joint configurations of robotic arms
    set_configuration!(state, q)
    set_configuration!(mvis, configuration(state))

    # Show table
    # material = MeshCat.MeshPhongMaterial(color=RGBA(1.0, 0.0, 0.0, 1.0))
    # setelement!(
    #     mvis, 
    #     root_frame(mechanism), 
    #     Rect3(Point(-0.4, 0.4, -0.15), Vec(1, 1, 0.1)), 
    #     material,
    #     "table")

    return mvis, mechanism
end

function generate_initial_guess(problem, goals, n, initial_state)
    g1, g2 = goals
    x1, x2 = initial_state
    guess = vcat(fill(x1, n)..., fill(zeros(r1.dof), n - 1)..., fill(x2, n)..., fill(zeros(r2.dof), n - 1)...)
    vcat(guess, zeros(total_dim(problem) - length(guess)))
end

function play_game(
    obj1::Vector{<:Real}=[0.5, 0, 0.1],
    obj2::Vector{<:Real}=[1.2, 0.3, 0.1],
    initial_state=mortar([zeros(r1.dof), zeros(r2.dof)]);
    Δt=0.5, T=1
)
    dof = r1.dof
    n = Int64(T / Δt) + 1
    goal = mortar([ik!(r1, obj1), ik!(r2, obj2)])
    println("R1 goal joint position = $(goal[Block(1)])")
    println("R2 goal joint position = $(goal[Block(2)])")
    println("Constructing problem...")
    problem = create_dynamic_game(goal; Δt=Δt, T=T)
    println("Solving problem...")
    (; primals, variables, status, info) = solve(
        problem, initial_state;
        initial_guess=generate_initial_guess(problem, blocks(goal), n, initial_state)
    )
    println(status)
    println(info)
    x = BlockArray(primals, dof * [n, n - 1, n, n - 1])
    return x[Block(1)], x[Block(3)]
end

function mpc(goals; Δt=1, T=2)
    traj1 = [zeros(r1.dof)]
    traj2 = [zeros(r2.dof)]
    for t in 0:Δt:T-Δt
        println("@ t = $t")
        x1, x2 = play_game(goals..., vcat(traj1[end], traj2[end]); Δt=Δt, T=T - t)
        println("x1 = $x1")
        println("x2 = $x2")
        push!(traj1, x1[r1.dof+1:2r1.dof])
        push!(traj2, x2[r2.dof+1:2r2.dof])
    end
    return traj1, traj2
end

function show_animation(goals, trajs, ts)
    goal1, goal2 = goals
    traj1, traj2 = trajs

    mvis, mechanism = visualize_robot()
    open(mvis)

    material = MeshCat.MeshPhongMaterial(color=RGBA(0.0, 1.0, 0.0, 0.9))
    setelement!(mvis, root_frame(mechanism), HyperSphere(Point(goal1...), 0.07), material, "g1")
    setelement!(mvis, root_frame(mechanism), HyperSphere(Point(goal2...), 0.07), material, "g2")

    qs = [[traj1[4ii-3:4ii] traj2[4ii-3:4ii]]'[:] for ii in 1:length(ts)]
    println("R1 final joint position = $(traj1[end])")
    println("R2 final joint position = $(traj2[end])")
    # println("R1 EE = $(forward(r1, traj1[end], r1.dof, r1.ee))")
    # println("R2 EE = $(forward(r2, traj2[end], r2.dof, r2.ee))")
    # animation = Animation(mvis, ts, qs; fps=25)
    # setanimation!(mvis, animation)
    MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.0)
    return mvis, ts, qs
end

function generate_object(robot)
    x = rand() + 0.3
    y = rand() - 0.2
    z = 0.05
    q = ik!(robot, [x, y, z])
    while maximum(forward(robot, q) - [x, y, z]) > 0.01
        x = rand() + 0.3
        y = rand() - 0.2
    end
    return [x, y, z]
end

# mutable struct OpenLoopStrategy
#     parametric_game::ParametricOptimizationProblem
#     strategy::Any = nothing
#     players::Any = [r1, r2]
#     state::Any = nothing
#     tasks::Any = [0, 0] # 0 for grasping and 1 for dropping
#     time_last_updated::Int = 0
#     turn_length::Int
#     horizon::Int
#     last_solution::Any = nothing
#     context_state::Any = nothing
#     solution_status::Any = nothing
# end

# function update_tasks(strategy::OpenLoopStrategy)
#     if 
# end

function main(problem=nothing, Δt=1, T=2)
    seed!(0)
    @time begin
        if isnothing(problem)
            # Construct game
            problem = create_dynamic_game(; Δt, T)
        end
        players = [r1, r2]
        n = Int64(T / Δt) + 1

        # Number of objects each robot should grasp
        n_round = 3
        # Count number of rounds finished already
        counts = [0, 0]

        # Locations of buckets for dropping objects
        drop_locations = [[0.0, -1.0, 0.0], [1.5, 0.8, 0]]

        # Generate initial objects
        goals_cartesian = [generate_object(players[ii]) for ii in 1:2]
        goals = [ik!(players[ii], goals_cartesian[ii]) for ii in 1:2]


        # Initial state of robots
        state = [zeros(r1.dof), zeros(r2.dof)]
        trajs = [state[1], state[2]]

        prev_state = nothing

        total_time = 0

        while maximum(counts) < n_round
            total_time += T
            println("Goal1($(goals[1])), Goal2($(goals[2]))")
            println("g1=$(goals_cartesian[1]), g2=$(goals_cartesian[2])")
            parameter_value = mortar([state..., goals...])
            (; primals, variables, status, info) = solve(
                problem, parameter_value;
            )

            # Update states
            x = BlockArray(primals, 4 * [n, n - 1, n, n - 1])
            prev_state = state
            state = [x[Block(1)][end-3:end], x[Block(3)][end-3:end]]
            # Update goals
            for ii in 1:2
                append!(trajs[ii], x[Block(2ii - 1)][5:end])
                if counts[ii] < n_round && maximum(forward(players[ii], state[ii]) - goals_cartesian[ii]) <= 0.01
                    if goals_cartesian[ii] == drop_locations[ii]
                        println("Player $ii have finished round $(counts[ii]+=1)")
                        goals_cartesian[ii] = generate_object(players[ii])
                    else
                        println("Player $ii have grasped the object")
                        goals_cartesian[ii] = drop_locations[ii]
                    end
                    goals[ii] = ik!(players[ii], goals_cartesian[ii])
                end
            end
        end

        ts = Vector{Float64}(0:Δt:total_time)
        mvis, ts, qs = show_animation(drop_locations, trajs, ts)
        return mvis, ts, qs, trajs
    end
end