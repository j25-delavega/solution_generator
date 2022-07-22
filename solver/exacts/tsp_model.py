import cplex
from cplex.exceptions import CplexError


def count(num_nodes, i, j):
    return num_nodes * i + j


def arc_cost_data(data):
    cost = []
    distances = data['euclidean_distances']

    for i in range(data['noNodes'] + 1):
        cost.append(0.0)  # distances[0][i]
    cost.append(0.0)

    for i in range(1, data['noNodes'] + 1):
        for j in range(data['noNodes'] + 1):
            cost.append(distances[i][j])
        cost.append(distances[i][0])

    for i in range(data['noNodes'] + 1):
        cost.append(0.0)  # distances[0][i]
    cost.append(0.0)

    return cost


def create_the_model(model, x_variable, p_variable, data, num_nodes):
    arc_cost = arc_cost_data(data)
    model.objective.set_sense(model.objective.sense.minimize)

    # Create variables x(i,j) for (i,j) in A
    for i in range(num_nodes):
        x_variable.append([])
        for j in range(num_nodes):
            var_count = count(num_nodes, i, j)
            var_name = "x." + str(i) + "." + str(j)
            x_variable[i].append(model.variables.get_num())
            model.variables.add(obj=[arc_cost[var_count]], lb=[0.0], ub=[1.0], types=["B"], names=[var_name])
        model.variables.set_upper_bounds(x_variable[i][i], 0)

    # Create variables p(i) for (i) in N
    for i in range(num_nodes):
        var_name = "p." + str(i)
        p_variable.append(model.variables.get_num())
        if i == 0:
            model.variables.add(obj=[0.0], lb=[0.0], ub=[0.0], types=["C"], names=[var_name])
        elif i == num_nodes - 1:
            model.variables.add(obj=[0.0], lb=[num_nodes - 1], ub=[num_nodes - 1], types=["C"], names=[var_name])
        else:
            model.variables.add(obj=[0.0], lb=[1.0], ub=[num_nodes - 1], types=["C"], names=[var_name])

    # Add the out degree constraints.
    # for all i in V: sum((i,j) in delta+(i)) x(i,j) = 1
    '''for j in range(1, num_nodes - 1):
        the_vars = []
        the_coefficients = []
        for i in range(0, num_nodes - 1):
            if i != j:
                the_vars.append(x_variable[i][j])
                the_coefficients.append(1.0)
        model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["E"], rhs=[1.0])'''

    # Add the out degree constraints.
    # for all i in V: sum((i,j) in delta+(i)) x(i,j) = 1
    for i in range(1, num_nodes - 1):
        the_vars = []
        the_coefficients = []
        for j in range(1, num_nodes):
            if i != j:
                the_vars.append(x_variable[i][j])
                the_coefficients.append(1)
        model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["E"], rhs=[1.0])

    for i in range(1, num_nodes - 1):
        the_vars = []
        the_coefficients = []
        for j in range(0, num_nodes - 1):
            if j != i:
                the_vars.append(x_variable[j][i])
                the_coefficients.append(1.0)
        for j in range(1, num_nodes):
            if j != i:
                the_vars.append(x_variable[i][j])
                the_coefficients.append(-1.0)
        model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["E"], rhs=[0.0])

    for i in range(1, num_nodes - 1):
        for j in range(1, num_nodes - 1):
            if i != j:
                the_vars = []
                the_coefficients = []

                the_vars.append(p_variable[i])
                the_coefficients.append(1.0)

                the_vars.append(p_variable[j])
                the_coefficients.append(-1.0)

                the_vars.append(x_variable[i][j])
                the_coefficients.append(num_nodes - 1.0)
                model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["L"],
                                             rhs=[(num_nodes - 2.0)])


def build_route(solution, x_variable, num_nodes):
    # Write out the optimal tour
    route = []
    successor = [-1] * num_nodes
    for i in range(num_nodes):
        sol = solution.get_values(x_variable[i])
        for j in range(num_nodes):
            if sol[j] > 1e-03:
                successor[i] = j
    print("Optimal tour:")
    i = 0
    while successor[i] != -1:  # and (successor[i] != num_nodes - 1):
        print("%d, " % i, end=' ')
        if i != num_nodes - 1:
            route.append(i)
        i = successor[i]
    print(i)
    return route


def printer(solution, num_nodes, x_variable):
    print()
    print("Solution status: ", solution.get_status())
    print("Objective value: ", solution.get_objective_value())

    if solution.get_status() == solution.status.MIP_optimal or \
            solution.get_status() == solution.status.MIP_time_limit_feasible:
        if solution.get_status() == solution.status.MIP_optimal:
            print("Solution status is optimal")
        if solution.get_status() == solution.status.MIP_time_limit_feasible:
            print("Solution status is feasible")
        return build_route(solution, x_variable, num_nodes)
    else:
        print("Solution status is infeasible or unbounded ")
        return []


def create_and_solve_the_model(data):
    try:
        model = cplex.Cplex()
        x_variable = []
        p_variable = []
        num_nodes = data['noNodes'] + 2
        create_the_model(model, x_variable, p_variable, data, num_nodes)
        model.parameters.threads.set(1)
        model.parameters.timelimit.set(100)
        model.solve()
        return printer(model.solution, num_nodes, x_variable)
    except CplexError as exc:
        raise

# if __name__ == "__main__":
#    create_and_solve_the_model()
