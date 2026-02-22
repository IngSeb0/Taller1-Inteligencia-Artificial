from typing import Any, Tuple
from algorithms import utils
from algorithms.problems import MultiSurvivorProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.
    Calculates the sum of absolute differences in coordinates.
    """
    xy1 = state
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.
    Calculates the straight-line distance between two points.
    """
    xy1 = state
    xy2 = problem.goal
    return ((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2) ** 0.5


def survivorHeuristic(state: Tuple[Tuple, Any], problem: MultiSurvivorProblem):
  """

    state: (position, survivors_grid)
    problem: MultiSurvivorProblem instance

    This must be admissible and preferably consistent.
    """
    posicion, sobrevivientes = state
    
    # Si no quedan sobrevivientes, la heurística es 0
    sobrevivientes_restantes = sobrevivientes.asList()
    if len(sobrevivientes_restantes) == 0:
        return 0
    
    # Calcular la distancia Manhattan al sobreviviente más cercano
    distancia_minima = float('inf')
    for sobreviviente in sobrevivientes_restantes:
        distancia = abs(posicion[0] - sobreviviente[0]) + abs(posicion[1] - sobreviviente[1])
        if distancia < distancia_minima:
            distancia_minima = distancia
    
    return distancia_minima