# 4-objective integer linear problem
#
# Example from vOptGeneric.jl tests

using JuMP; using GLPK; using KirlikSayin

function main()
    m = Model(GLPK.Optimizer); set_silent(m)
    @variable(m, 0 <= x[1:4] <= 2, Int)
    @constraint(m, sum(x) <= 3)
    expr1 = @expression(m, -sum(x))
    expr2 = @expression(m, sum(x))
    expr3 = @expression(m, -x[3])
    expr4 = @expression(m, -x[4])
    Yn = get_pareto_front(m, [expr1, expr2, expr3, expr4])
    return Yn
end
