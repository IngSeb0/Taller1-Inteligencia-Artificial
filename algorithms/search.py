from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyHouseSearch(problem: SearchProblem):
    """
    Returns a sequence of moves that solves tinyHouse. For any other building, the
    sequence of moves will be incorrect, so only use this for tinyHouse.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    
    pila = utils.Stack()
    visitados = set()

    prim_estado = (problem.getStartState(), [])
    pila.push(prim_estado)

    while not pila.isEmpty():
        estado, camino = pila.pop()
        if problem.isGoalState(estado):
            return camino

        if estado not in visitados:
            visitados.add(estado)

            succ = problem.getSuccessors(estado)
            for i, mov, cost in succ:
                pila.push((i, camino+[mov]))

    return []


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    cola = utils.Queue()
    visitados = set()

    prim_estado = (problem.getStartState(), [])
    cola.push(prim_estado)

    while not cola.isEmpty():
        estado, camino = cola.pop()
        if problem.isGoalState(estado):
            return camino

        if estado not in visitados:
            visitados.add(estado)

            succ = problem.getSuccessors(estado)
            for i, mov, cost in succ:
                cola.push((i, camino+[mov]))

    return []


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """
    puntos= utils.PriorityQueue()
    estado_I= problem.getStartState()
    puntos.push((estado_I,[],0),0)
    visitado={}
    while puntos.isEmpty()!= True:
        estado, pasos, costo= puntos.pop()
        if estado in visitado and visitado[estado] <= costo:
            continue
        visitado[estado]=costo
        if problem.isGoalState(estado):
            return pasos
        for sig, paso, n_costo in problem.getSuccessors(estado):
            sum= costo+n_costo
            pasos_sig= pasos+[paso] 
            puntos.push((sig, pasos_sig, sum), sum)
    return[]
            
    


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    
    This implements A* search using:
    f(n) = g(n) + h(n)
    where:
    - g(n) is the actual cost from start to node n
    - h(n) is the heuristic estimate from n to goal
    """
    
   
    startState = problem.getStartState()
    startHeuristic = heuristic(startState, problem)
    startCost = 0
    
    frontier = utils.PriorityQueue()
    frontier.push((startState, []), startCost + startHeuristic)
    
    visited = set()
    bestCost = {startState: startCost}
    
    while not frontier.isEmpty():
        state, actions = frontier.pop()
        
        if state in visited:
            continue
        visited.add(state)
        
        if problem.isGoalState(state):
            return actions
        
        for successor, action, stepCost in problem.getSuccessors(state):
            newCost = bestCost[state] + stepCost
            
            if successor not in bestCost or newCost < bestCost[successor]:
                bestCost[successor] = newCost
                heuristicValue = heuristic(successor, problem)
                fValue = newCost + heuristicValue
                newActions = actions + [action]
                frontier.push((successor, newActions), fValue)
    
    return []


# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
