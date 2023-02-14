**Deprecated in favor of [MultiObjectiveAlgorithms](https://github.com/jump-dev/MultiObjectiveAlgorithms.jl).**

# Kirlik and Sayin
"[This] method uses the well-established [epsilon]-constraint scalarization and is based on a partitioning mechanism that searches the (p-1)-dimensional constraint space exhaustively." -- Ref: [https://doi.org/10.1016/j.ejor.2013.08.001](https://doi.org/10.1016/j.ejor.2013.08.001) (2014)

This package aim at providing Pareto front using the Kirlik and Sayin approach from 2014.



### Usage:
This package is not registered and should be added via
```julia
]add git@github.com:remi-garcia/KirlikSayin.jl.git
```

```julia
get_pareto_front(model, expressions_objective_functions)
```
where `model` is a JuMP model and `expressions_objective_functions` is a `Vector{X} where X <: AbstractJuMPScalar` for the multiple objective functions.

See `examples/basicvopt.jl` for a simple example.



### Implementation details
- In the original algorithm, rectangles are stored in (p-1) dimension. In our implementation these are stored in the p-dimension and procedures and functions loops start at $i=2$.
- We believe there is a typo in the original paper. In our understanding, the original algorithm, as presented in the paper, ends in an infinite loop: the `updateRect` function does not remove the first obtained solution from any rectangle, hence nothing prevent that solution to be obtained again and again after each `solveModel` call. We prevent that by replacing the definition of
$\hat{R} = \\{\overline{y} \in R\_t : \overline{y}\_j \leq \overline{f}(x^{\*})\_j\\}$
by
$\hat{R} = \\{\overline{y} \in R\_t : \overline{y}\_j < \overline{f}(x^{\*})\_j\\}$.
- Similarly, in `updateList`, we replaced the condition
$l\_{ij} < \overline{f}\_{j}(x^{\*}) < u\_{ij}$
by
$l\_{ij} < \overline{f}\_{j}(x^{\*}) \leq u\_{ij}$.
Otherwise, the risk of cycling remains.



### Other implementations
- [vongostev/QPIP](https://github.com/vongostev/QPIP) -- python
- [dbomahn/savedHeuristicBen](https://github.com/dbomahn/phd/blob/master/codes/savedHeuristicBen.jl) -- C++
- [ArthurGontierPro/KetS](https://github.com/ArthurGontierPro/Master-Archives/blob/master/meta_m2-master/Src/KetS.jl) -- julia
