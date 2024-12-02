using FinalProject
using BlockArrays
using ColorTypes
using Infiltrator
using LinearAlgebra: norm_sqr
using RigidBodyDynamics
using MeshCat
using MeshCatMechanisms
using GeometryBasics
using StaticArrays
using Random: seed!
using Serialization

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

    # Show table
    if table
        material = MeshCat.MeshPhongMaterial(color=RGBA(1.0, 0.0, 0.0, 0.3))
        setelement!(
            mvis,
            root_frame(mechanism),
            Rect3(Point(0.3, -0.3, 0.0), Vec(1.0, 1.0, 0.0)),
            material,
            "table"
        )
    end

    return mvis, mechanism
end

function generate_initial_guess(problem, state, goals, n)
    g1, g2 = goals
    x1, x2 = state
    guess = vcat(fill(x1, n)..., fill(zeros(r1.dof), n - 1)..., fill(x2, n)..., fill(zeros(r2.dof), n - 1)...)
    vcat(guess, zeros(total_dim(problem) - length(guess)))
end

function m_simulate(trajs, ts)
    traj1, traj2 = trajs
    qs = [[traj1[4ii-3:4ii] traj2[4ii-3:4ii]]'[:] for ii in 1:length(ts)]
    println("R1 final joint position = $(traj1[end])")
    println("R2 final joint position = $(traj2[end])")
    # println("R1 EE = $(forward(r1, traj1[end], r1.dof, r1.ee))")
    # println("R2 EE = $(forward(r2, traj2[end], r2.dof, r2.ee))")
    return qs
end

function generate_object(robot)
    x = rand() * 0.8 + 0.4
    y = rand() * 0.8 - 0.4
    z = 0.05
    q = ik!(robot, [x, y, z])
    while maximum(forward(robot, q) - [x, y, z]) > 0.01
        x = rand() + 0.3
        y = rand() - 0.2
    end
    return [x, y, z]
end

function generate_object(robot, mvis, frame, color, label)
    coord = generate_object(robot)
    material = MeshCat.MeshPhongMaterial(color=color)
    setelement!(mvis, frame, HyperSphere(Point(coord...), 0.075), material, label)
    return coord
end

function get_problem(filename=nothing)
    if isnothing(filename)
        problem = create_dynamic_game(; Δt=0.5, T=1)
        serialize("ParametricMCPProblem", problem)
    else
        problem = deserialize(filename)
    end
    return problem
end

function main(problem=nothing, Δt=0.5, T=1)
    seed!(4)
    @time begin
        if isnothing(problem)
            # Construct game
            problem = create_dynamic_game(; Δt, T)
        end

        mvis, mechanism = visualize_robot(; bubbles=true, table=false)

        players = [r1, r2]
        colors = [RGBA(1.0, 1.0, 0.0, 1.0), RGBA(1.0, 0.0, 1.0, 1.0)]
        n = Int64(T / Δt) + 1

        # Number of objects each robot should grasp
        n_round = 3
        # Count number of rounds finished already
        counts = [0, 0]

        # Locations of buckets for dropping objects
        drop_locations = [[0.0, -1.0, 0.0], [1.8, 0.8, 0]]
        material = MeshCat.MeshPhongMaterial(color=RGBA(0.0, 1.0, 1.0, 0.5))
        setelement!(mvis, root_frame(mechanism), HyperSphere(Point(drop_locations[1]...), 0.1), material, "Drop 1")
        setelement!(mvis, root_frame(mechanism), HyperSphere(Point(drop_locations[2]...), 0.1), material, "Drop 2")

        # Generate initial objects
        goals_cartesian = [generate_object(players[ii], mvis, root_frame(mechanism), colors[ii], "g$ii (#1)") for ii in 1:2]
        goals = [ik!(players[ii], goals_cartesian[ii]) for ii in 1:2]


        # Initial state of robots
        state = [zeros(r1.dof), zeros(r2.dof)]
        trajs = [state[1], state[2]]

        total_time = 0
        tol = 1e-4

        is_stuck = false

        while !is_stuck && minimum(counts) < n_round
            total_time += T
            println("time = $total_time s")
            println("Goal1($(goals[1])), Goal2($(goals[2]))")
            println("g1=$(goals_cartesian[1]), g2=$(goals_cartesian[2])")
            parameter_value = mortar([state..., goals...])
            (; primals, variables, status, info) = solve(
                problem, parameter_value;
                initial_guess=generate_initial_guess(problem, state, goals, n)
            )

            # Update states
            x = BlockArray(primals, 4 * [n, n - 1, n, n - 1])
            prev_state = state
            state = [x[Block(1)][end-3:end], x[Block(3)][end-3:end]]
            # @infiltrate

            # Update goals
            for ii in 1:2
                if counts[ii] < n_round && norm_sqr(prev_state[ii] - state[ii]) < tol
                    is_stuck = true
                    println("Player $ii is stucked!!!")
                end
                append!(trajs[ii], x[Block(2ii - 1)][5:end])
                # @infiltrate
                if counts[ii] < n_round && norm_sqr(forward(players[ii], state[ii]) - goals_cartesian[ii]) <= tol
                    if goals_cartesian[ii] == drop_locations[ii]
                        println("Player $ii have finished round $(counts[ii]+=1)")
                        if counts[ii] < n
                            goals_cartesian[ii] = generate_object(players[ii], mvis, root_frame(mechanism), colors[ii], "g$ii (#$(counts[ii]+1))")
                        end
                    else
                        println("Player $ii have grasped the object")
                        goals_cartesian[ii] = drop_locations[ii]
                    end
                    goals[ii] = ik!(players[ii], goals_cartesian[ii])
                end
            end
        end

        ts = Vector{Float64}(0:Δt:total_time)
        qs = m_simulate(trajs, ts)
        open(mvis)
        return mvis, ts, qs
    end
end

# MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.0)
