## Setup

```console
julia> ]
pkg> activate .
julia> using Revise
julia> using FinalProject
julia> problem = create_dynamic_game(; Δt=1, T=2)
julia> include("examples/demo.jl")
julia> mvis, ts, qs, trajs = main(problem)
```
Refresh browser MeshCat page
```console
julia> MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.)
```