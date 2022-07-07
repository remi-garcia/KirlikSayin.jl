function updateList(L::Vector{MVector{N, Tuple{Int, Int}}}, current_solution::MVector{N, Int}) where N
    Lprime = deepcopy(L)
    L = Vector{MVector{N, Tuple{Int, Int}}}()
    for Ri in Lprime
        T = Vector{MVector{N, Tuple{Int, Int}}}([Ri])
        for j in 2:N
            if Ri[j][1] < current_solution[j] && current_solution[j] <= Ri[j][2]
                Tprime = Vector{MVector{N, Tuple{Int, Int}}}()
                for Rt in T
                    push!(Tprime, MVector{N, Tuple{Int, Int}}([[(Rt[i][1], Rt[i][2]) for i in 1:(j-1)];[(Rt[j][1], current_solution[j]-1)];[(Rt[i][1], Rt[i][2]) for i in (j+1):N]]))
                    push!(Tprime, MVector{N, Tuple{Int, Int}}([[(Rt[i][1], Rt[i][2]) for i in 1:(j-1)];[(current_solution[j], Rt[j][2])];[(Rt[i][1], Rt[i][2]) for i in (j+1):N]]))
                end
                T = deepcopy(Tprime)
            end
        end
        append!(L, T)
    end
    return L
end


function isIncluded(R1::MVector{N, Tuple{Int, Int}}, R2::MVector{N, Tuple{Int, Int}}) where N
    for i in 2:N
        if R1[i][1] < R2[i][1] || R1[i][2] > R2[i][2]
            return false
        end
    end
    return true
end


function removeRect!(L::Vector{MVector{N, Tuple{Int, Int}}}, lb::MVector{N, Int}, ub::MVector{N, Int}) where N
    to_remove = falses(length(L))
    for i in 1:length(L)
        if isIncluded(L[i], MVector{N, Tuple{Int, Int}}([(lb[i], ub[i]) for i in 1:N]))
            to_remove[i] = true
        end
    end
    deleteat!(L, to_remove)
    return L
end

function upper_corner(R::MVector{N, Tuple{Int, Int}}) where N
    return MVector{N, Int}([R[i][2] for i in 1:N])
end


function adjust_model!(model::Model, objective_functions::Vector{X}) where X <:AbstractJuMPScalar
    N = length(objective_functions)
    @variable(model, kirliksayin_objective_functions[i in 1:N], Int)
    @constraint(model, kirliksayin_objective_functions_csts[i in 1:N], kirliksayin_objective_functions[i] == objective_functions[i])
    return model
end


function fix_bounds_model!(model::Model, lb::MVector{N, Int}, ub::MVector{N, Int}) where N
    kirliksayin_objective_functions = model[:kirliksayin_objective_functions]
    for i in 1:N
        set_lower_bound(kirliksayin_objective_functions[i], lb[i])
        set_upper_bound(kirliksayin_objective_functions[i], ub[i])
    end
    return model
end


function cleanup_model!(model::Model)
    kirliksayin_objective_functions = model[:kirliksayin_objective_functions]
    N = length(kirliksayin_objective_functions)
    kirliksayin_objective_functions_csts = constraint_by_name.(model, ["kirliksayin_objective_functions_csts[$i]" for i in 1:N])
    delete.(model, kirliksayin_objective_functions_csts)
    unregister(model, :kirliksayin_objective_functions_csts)
    delete.(model, kirliksayin_objective_functions)
    unregister(model, :kirliksayin_objective_functions)
    return model
end


function solveModel(model::Model, ub::MVector{N, Int})::Union{Nothing, MVector{N, Int}} where N
    kirliksayin_objective_functions = model[:kirliksayin_objective_functions]

    @objective(model, Min, kirliksayin_objective_functions[1])
    @constraint(model, kirliksayin_objective_functions_eps[i in 2:N], kirliksayin_objective_functions[i] <= ub[i])
    optimize!(model)
    if termination_status(model) != OPTIMAL
        delete.(model, kirliksayin_objective_functions_eps)
        unregister(model, :kirliksayin_objective_functions_eps)
        return nothing
    end

    new_obj_value = round(Int, objective_value(model))
    @objective(model, Min, sum(kirliksayin_objective_functions[2:end]))
    @constraint(model, kirliksayin_objective_functions_equal, kirliksayin_objective_functions[1] == new_obj_value)
    optimize!(model)
    if termination_status(model) != OPTIMAL
        delete.(model, kirliksayin_objective_functions_eps)
        delete.(model, kirliksayin_objective_functions_equal)
        unregister(model, :kirliksayin_objective_functions_eps)
        unregister(model, :kirliksayin_objective_functions_equal)
        return nothing
    end

    objective_functions_values = MVector{N, Int}(round.(Int, value.(kirliksayin_objective_functions)))

    delete.(model, kirliksayin_objective_functions_eps)
    delete.(model, kirliksayin_objective_functions_equal)
    unregister(model, :kirliksayin_objective_functions_eps)
    unregister(model, :kirliksayin_objective_functions_equal)

    return objective_functions_values
end


function get_volume(R::MVector{N, Tuple{Int, Int}}, lb::MVector{N, Int}) where N
    return prod((R[i][2] - lb[i]) for i in 2:N)
end


function rargmax(L::Vector{MVector{N, Tuple{Int, Int}}}, lb::MVector{N, Int})::MVector{N, Tuple{Int, Int}} where N
    current_max_volume = -1
    current_volume = 0
    largest_R = 0
    for i in 1:length(L)
        current_volume = get_volume(L[i], lb)
        if current_volume > current_max_volume
            current_max_volume = current_volume
            largest_R = i
        end
    end
    return copy(L[largest_R])
end


function get_rbounds(model::Model, objective_functions::Vector{X}) where X <: AbstractJuMPScalar
    N = length(objective_functions)
    lb = zeros(MVector{N, Int})
    # ub = zeros(MVector{N, Int})

    kirliksayin_objective_functions = model[:kirliksayin_objective_functions]
    @objective(model, Min, kirliksayin_objective_functions[1])
    optimize!(model)
    lb[1] = round(Int, value(kirliksayin_objective_functions[1]))
    ub = MVector{N, Int}(round.(Int, value.(kirliksayin_objective_functions)))

    for i in 2:N
        @objective(model, Min, kirliksayin_objective_functions[i])
        optimize!(model)
        lb[i] = round(Int, value(kirliksayin_objective_functions[i]))
        for j in 1:N
            ub[j] = max(ub[j], round(Int, value(kirliksayin_objective_functions[j])))
        end
    end

    return lb, ub
end



function get_pareto_front(model::Model, objective_functions::Vector{X}) where X <: AbstractJuMPScalar
    N = length(objective_functions)
    # Adjust model to add variables for objective functions
    adjust_model!(model, objective_functions)
    lb, ub = get_rbounds(model, objective_functions)
    fix_bounds_model!(model, lb, ub)
    L = Vector{MVector{N, Tuple{Int, Int}}}([[(lb[i], ub[i]) for i in 1:N]])
    Yn = Vector{MVector{N, Int}}()
    while !isempty(L)
        Ri = rargmax(L, lb)
        ui = upper_corner(Ri)
        fxstar::Union{Nothing, MVector{N, Int}} = solveModel(model, ui)
        if !isnothing(fxstar)
            if !(fxstar in Yn)
                push!(Yn, fxstar)
                L = updateList(L, fxstar)
                removeRect!(L, fxstar, ui)
            else
                removeRect!(L, fxstar, ui)
            end
        else
            removeRect!(L, lb, ui)
        end
    end
    # Remove our changes
    cleanup_model!(model)
    return Yn
end
