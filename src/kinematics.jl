using NLsolve

mutable struct Robot
    dof::Integer
    αs::Vector{<:Real}
    as::Vector{<:Real}
    ds::Vector{<:Real}
    qs::Vector{<:Real}
    T₀::Matrix{<:Real}
    ee::Vector{<:Real}
end

function Robot(αs, as, ds, qs, ee, T₀=Matrix{Float64}(I(4)))
    @assert length(αs) == length(as) == length(ds) == length(qs)
    Robot(length(αs), αs, as, ds, qs, T₀, ee)
end

# Function to get the transformation matrix between two neigboring links
# Parameters are given in DH notation
# Output is a 4x4 matrix that transforms coordinates from frame i to frame i-1
function get_transformation_matrix(α::Real, a::Real, d::Real, θ::Real)
    return [
        cos(θ) -sin(θ) 0 a;
        sin(θ)*cos(α) cos(θ)*cos(α) -sin(α) -sin(α)*d;
        sin(θ)*sin(α) cos(θ)*sin(α) cos(α) cos(α)*d;
        0 0 0 1
    ]
end

function get_transformation_matrix(r::Robot, ii::Integer)
    get_transformation_matrix(r.αs[ii], r.as[ii], r.ds[ii], r.qs[ii])
end

function get_transformation_matrix(r::Robot, ii::Integer, x::Vector{<:Real})
    get_transformation_matrix(r.αs[ii], r.as[ii], r.ds[ii], r.qs[ii] + x[ii])
end

function forward(r::Robot, ind::Integer, p::Vector{<:Real})
    p = [p..., 1]
    (r.T₀*reduce(*, [get_transformation_matrix(r, ii) for ii in 1:ind])*p)[1:3]
end

function forward(r::Robot, x::Vector{<:Real}, ind::Integer, p::Vector{<:Real})
    p = [p..., 1]
    (r.T₀*reduce(*, [get_transformation_matrix(r, ii, x) for ii in 1:ind])*p)[1:3]
end

function ik!(r::Robot, ee::Vector{<:Real}, point::Vector{<:Float64})
    f(x) = forward(r, x, r.dof, ee) - point
    sol = nlsolve(f, [0., 0., 0., 0.])
    sol.zero
end

function ik!(state::MechanismState, body::RigidBody, point::Point3D, desired::Point3D, α=0.1, iterations=100)
    mechanism = state.mechanism
    world = root_frame(mechanism)

    # Compute the joint path from world to our target body
    p = path(mechanism, root_body(mechanism), body)
    # Allocate the point jacobian (we'll update this in-place later)
    Jp = point_jacobian(state, p, transform(state, point, world))

    q = copy(configuration(state))

    for _ in 1:iterations
        # Update the position of the point
        point_in_world = transform(state, point, world)
        # Update the point's jacobian
        point_jacobian!(Jp, state, p, point_in_world)
        # Compute an update in joint coordinates using the jacobian transpose
        Δq = α * Array(Jp)' * (transform(state, desired, world) - point_in_world).v
        # Apply the update
        q .= configuration(state) .+ Δq
        set_configuration!(state, q)
    end
    return state
end

function build_mechanism(urdf::String="src/urdf/4dof.urdf")
    parse_urdf(urdf, remove_fixed_tree_joints=false)
end