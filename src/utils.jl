export write_puma560_urdf
function write_puma560_urdf()
    puma_mesh_dir = joinpath("src", "urdf", "meshes")
    urdf_orig = joinpath("src", "urdf", "puma560_robot_original.urdf")
    urdf_temp = joinpath("src", "urdf", "temp", "puma560.urdf")
    urdf = joinpath("src", "urdf", "puma560.urdf")
    open(urdf_orig, "r") do f
        open(urdf_temp, "w") do fnew
            for line in eachline(f)
                pre = findfirst("<mesh filename=", line)
                post = findlast("/>", line)
                if !(pre isa Nothing) && !(post isa Nothing)
                    ii = pre[end]+2:post[1]-2
                    pathstr = line[ii]
                    file = splitdir(pathstr)[2]
                    line = line[1:pre[end]+1] * joinpath(puma_mesh_dir, file) * line[post[1]-1:end]
                end
                println(fnew, line)
            end
        end
    end
    cp(urdf_temp, urdf; force=true)
end