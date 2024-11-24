using FinalProject
using RigidBodyDynamics
using MeshCatMechanisms

function build_mechanism()
    urdf = "src/urdf/4dof.urdf"
    parse_urdf(urdf, remove_fixed_tree_joints=false)
end

function check_kinematics()
    # Using RigidBodyDynamics
    mechanism = build_mechanism()
    state = MechanismState(mechanism)

    # Using customed implemented robot kinematics
    dof = 4
    αs = [0, -pi/2, 0, 0]
    as = [0, 0, 0.4, 0.3]
    ds = [0.4, 0.1, 0., 0.]
    qs = [0., 0., pi/2, -pi/2]
    r = Robot(αs, as, ds, qs)

    # Check coordinates matched between the two models
    point = (1, 2, 3)
    for ii in 1:dof
        p1 = transform(state, Point3D(default_frame(findbody(mechanism, "link$(ii+1)")), point...), root_frame(mechanism))
        p2 = forward(r, ii, point)
        @assert maximum(p1.v - p2) < 0.01 "$(p1.v) != $p2"
    end

    println("Correct!")
end