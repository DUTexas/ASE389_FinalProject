export Robot
mutable struct Robot
    dof
    αs
    as
    ds
    qs
end

function Robot(αs, as, ds, qs)
    @assert length(αs) == length(as) == length(ds) == length(qs)
    Robot(length(αs), αs, as, ds, qs)
end

# Function to get the transformation matrix between two neigboring links
# Parameters are given in DH notation
# Output is a 4x4 matrix that transforms coordinates from frame i to frame i-1
export get_transformation_matrix
function get_transformation_matrix(α, a, d, θ)
    return [
        cos(θ)        -sin(θ)       0       a;
        sin(θ)*cos(α) cos(θ)*cos(α) -sin(α) -sin(α)*d;
        sin(θ)*sin(α) cos(θ)*sin(α) cos(α)  cos(α)*d;
        0             0             0       1
    ]
end

function get_transformation_matrix(r:: Robot, ii::Int64)
    get_transformation_matrix(r.αs[ii], r.as[ii], r.ds[ii], r.qs[ii])
end

export forward
function forward(r::Robot, ind::Int64, p::Tuple{Number, Number, Number})
    p = [p..., 1]
    (reduce(*, [get_transformation_matrix(r, ii) for ii in 1:ind]) * p)[1:3]
end