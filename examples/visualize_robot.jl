using FinalProject
using RigidBodyDynamics
using MeshCatMechanisms
using StaticArrays


function show_robot()
    # robot = create_puma_560()
    mechanism = create_two_puma_560()
    mvis = MechanismVisualizer(mechanism, Skeleton(inertias=false))
    # mvis = MechanismVisualizer(robot, URDFVisuals("src/urdf/puma560.urdf"))
    # mvis = MechanismVisualizer(mechanism, URDFVisuals("src/urdf/puma560_two.urdf"))
    open(mvis)
    for body in bodies(mechanism)
        setelement!(mvis, default_frame(body))
    end

    setelement!(mvis, Point3D(default_frame(findbody(mechanism, "link3")), 0.2, 0., 0.25), 0.1)
    setelement!(mvis, Point3D(default_frame(findbody(mechanism, "link4")), 0., 0., 0.), 0.1)
    setelement!(mvis, Point3D(default_frame(findbody(mechanism, "link5")), 0., 0., 0.2), 0.1)
    setelement!(mvis, Point3D(default_frame(findbody(mechanism, "link6")), 0., 0., 0.), 0.1)


    # state = MechanismState(mechanism)
    # ee1 = findbody(mechanism, "link7")
    # point = Point3D(default_frame(ee1), 0., 0., 0.)
    # tar = Point3D(root_frame(mechanism), 0.5, 0.2, 0.2)
    # ik!(state, ee1, point, tar)
    # set_configuration!(mvis, configuration(state))
    # transform(state, point, root_frame(mechanism))
end

check_kinematics()