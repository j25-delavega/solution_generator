import cplex
from cplex.callbacks import LazyConstraintCallback
from cplex.exceptions import CplexError
from output.output_format import output_file
from utils.utils import build_output


# The class BendersLazyConsCallback
# allows to add Benders' cuts as lazy constraints.
#
class CustomLazyConsCallback(LazyConstraintCallback):

    def __init__(self, env):
        super().__init__(env)
        self.data = None
        self.name = None
        self.seed = None
        self.x_variable = None
        self.count = 0

    def __call__(self):
        print(" >> Hello LazyCallback ")
        x_variable = self.x_variable
        num_nodes = len(x_variable)
        seed = self.seed + self.count

        route = build_route(self, x_variable, num_nodes)
        print(' >> points in the route: ' + str(len(route)))
        build_output(output_file(seed, route, self.data), self.name)
        self.count += 1


def count(num_nodes, i, j):
    return num_nodes * i + j


def arc_cost_data(data):
    cost = []
    distances = []
    for i in range(data['noNodes'] + 1):
        values = []
        for j in range(data['noNodes'] + 1):
            values.append(data['real_distances'][i][j] + 100 * \
                          data['euclidean_distances'][i][j])
        distances.append(values)

    for i in range(data['noNodes'] + 1):
        cost.append(0.0)  # distances[0][i]
    cost.append(0.0)

    for i in range(1, data['noNodes'] + 1):
        values = distances[i]
        for j in range(data['noNodes'] + 1):
            cost.append(values[j])
        cost.append(values[0])

    for i in range(data['noNodes'] + 1):
        cost.append(0.0)  # distances[0][i]
    cost.append(0.0)

    return cost


def flow_constraints(model, x_variable, num_nodes):
    # Add the out degree constraints.
    # for all i in V: sum((i,j) in delta+(i)) x(i,j) = 1
    for v in range(1, num_nodes - 1):
        the_vars = []
        the_coefficients = []
        for i in range(0, num_nodes - 1):
            if i != v:
                the_vars.append(x_variable[i][v])
                the_coefficients.append(1)
        model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["E"], rhs=[1.0])

    # Add the out degree constraints.
    # for all i in V: sum((i,j) in delta+(i)) x(i,j) = 1
    for v in range(1, num_nodes - 1):
        the_vars = []
        the_coefficients = []
        for j in range(1, num_nodes):
            if j != v:
                the_vars.append(x_variable[v][j])
                the_coefficients.append(1)
        model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["E"], rhs=[1.0])


def depot_constraints(model, x_variable, num_nodes):
    the_vars = []
    the_coefficients = []
    for i in range(1, num_nodes - 1):
        the_vars.append(x_variable[0][i])
        the_coefficients.append(1)
    model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["E"], rhs=[1.0])

    the_vars = []
    the_coefficients = []
    for i in range(1, num_nodes - 1):
        the_vars.append(x_variable[i][num_nodes - 1])
        the_coefficients.append(1)
    model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["E"], rhs=[1.0])


def mtz_constraints(model, x_variable, p_variable, num_nodes):
    for j in range(1, num_nodes - 1):
        for i in range(1, num_nodes - 1):
            if i != j:
                the_vars = []
                the_coefficients = []

                the_vars.append(p_variable[j])
                the_coefficients.append(-1)

                the_vars.append(p_variable[i])
                the_coefficients.append(1)

                the_vars.append(x_variable[i][j])
                the_coefficients.append(num_nodes)
                model.linear_constraints.add(lin_expr=[cplex.SparsePair(the_vars, the_coefficients)], senses=["L"],
                                             rhs=[num_nodes - 1])


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
        model.variables.set_upper_bounds(x_variable[i][i], 0.0)

    # Create variables p(i) for (i) in N
    for i in range(num_nodes):
        var_name = "p." + str(i)
        p_variable.append(model.variables.get_num())
        model.variables.add(obj=[0.0], lb=[1.0], ub=[num_nodes], names=[var_name])

    flow_constraints(model, x_variable, num_nodes)
    depot_constraints(model, x_variable, num_nodes)
    mtz_constraints(model, x_variable, p_variable, num_nodes)


def build_route(solution, x_variable, num_nodes):
    # Write out the optimal tour
    route = []
    successor = [-1] * num_nodes
    for i in range(num_nodes):
        sol = solution.get_values(x_variable[i])
        for j in range(num_nodes):
            if sol[j] > 1e-03:
                successor[i] = j
    i = 0
    while successor[i] != -1:
        if i != num_nodes - 1:
            route.append(i)
        i = successor[i]
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


def create_and_solve_the_model(data, name, seed):
    try:
        model = cplex.Cplex()
        x_variable = []
        p_variable = []
        num_nodes = data['noNodes'] + 2
        create_the_model(model, x_variable, p_variable, data, num_nodes)
        model.parameters.threads.set(1)
        model.parameters.timelimit.set(28800)
        lazy_benders = model.register_callback(CustomLazyConsCallback)
        lazy_benders.x_variable = x_variable
        lazy_benders.solution = model.solution
        lazy_benders.seed = seed
        lazy_benders.name = name
        lazy_benders.data = data
        model.solve()
        return printer(model.solution, num_nodes, x_variable)
    except CplexError as exc:
        raise
