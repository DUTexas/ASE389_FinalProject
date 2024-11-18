using Infiltrator
using MeshCatMechanisms


export create_puma_560
function create_puma_560()
    urdf = "src/urdf/puma560.urdf"
    return parse_urdf(urdf, remove_fixed_tree_joints=false)
end

export create_two_puma_560
function create_two_puma_560()
    urdf = "src/urdf/puma560_two.urdf"
    return parse_urdf(urdf, remove_fixed_tree_joints=false)
end

export ik!
# Inverse kinematics
function ik!(state::MechanismState, body::RigidBody, point::Point3D, desired::Point3D, α=0.1, iterations=100)
    mechanism = state.mechanism
    world = root_frame(mechanism)

    # Compute the joint path from world to our target body
    p = path(mechanism, root_body(mechanism), body)
    # Allocate the point jacobian (we'll update this in-place later)
    Jp = point_jacobian(state, p, transform(state, point, world))

    q = copy(configuration(state))

    for i in 1:iterations
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

export collision_bubbles
# Generate two arrays of points representing centers of collision bubbles in world coordinates
function collision_bubbles(mechanism::Mechanism)
    radius = 0.3
    body_names1 = ["link4", "link5", "link6", "link7"]
    body_names2 = ["link4_2", "link5_2", "link6_2", "link7_2"]
    coords = [(0.2, 0., 0.25), (0., 0., 0.), (0., 0., 0.2), (0., 0., 0.)]
    bubbles1 = []
    bubbles2 = []

    ground = root_frame(mechanism)
    state = MechanismState(mechanism)
    for ii in eachindex(body_names1)
        body = findbody(mechanism, body_names1[ii])
        point = transform(state, Point3D(default_frame(body), coords[ii]...), ground)
        push!(bubbles1, point.v)
        body = findbody(mechanism, body_names2[ii])
        point = transform(state, Point3D(default_frame(body), coords[ii]...), ground)
        push!(bubbles2, point.v)
    end
    
    return [bubbles1, bubbles2]
end