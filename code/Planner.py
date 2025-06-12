import numpy as np
import astar
import ompl.base as ob
import ompl.geometric as og
import collision_checker   

class MyPlanner:
  __slots__ = ['boundary', 'blocks']

  def __init__(self, boundary, blocks):
    self.boundary = boundary.reshape(-1)
    self.blocks = blocks


  def plan_search(self, start, goal):
    path = astar.AStar.plan(
        start, goal,
        self.boundary,
        self.blocks,
        resolution=0.5,
        epsilon=5.0
    )
    return path
      
  
  def plan_sample(self,
                start: np.ndarray,
                goal : np.ndarray,
                *,
                algo: str = 'RRTConnect',
                runtime: float = 2.0,
                simplify: bool = True,
                **planner_kwargs) -> np.ndarray:
    """Plan with any OMPL geometric planner and return an ``(N,3)`` NumPy path.

    Parameters
    ----------
    start, goal : np.ndarray
        Cartesian 3‑vector of start / goal positions.
    algo : str, default ``"RRTConnect"``
        Planner name – e.g. ``"RRT"``, ``"BITstar"``, ``"PRMstar"``.
    runtime : float, default ``2.0``
        Maximum planning time in **seconds**.
    simplify : bool, default ``True``
        After a solution is found, call OMPL's path simplifier.
    resolution : float | None
        If given, set the state‑validity checking resolution to this fraction
        of the space's maximum extent (helps avoid edge collisions).
    planner_kwargs : dict
        Extra arguments forwarded to the specific planner, e.g.
        ``range=0.5`` for RRT.
    """
    # --- 1. Build a 3‑D real‑vector state space with axis‑aligned bounds ------
    space = ob.RealVectorStateSpace(3)
    bounds = ob.RealVectorBounds(3)
    low, high = self.boundary[:3], self.boundary[3:]
    for i in range(3):
        bounds.setLow(i,  float(low[i]))
        bounds.setHigh(i, float(high[i]))
    space.setBounds(bounds)
    
    # --- 2. Space information + validity & motion checkers --------------
    si = ob.SpaceInformation(space)
    si.setStateValidityCheckingResolution(0.0001)  # 1% of Maximum diameter

    class _VC(ob.StateValidityChecker):
        def __init__(self, si_, boundary6, boxes6):
            super().__init__(si_)
            self._boundary = boundary6        # shape (6,)
            self._boxes    = boxes6           # shape (N,6)

        def isValid(self, state):
            pt = np.array([state[i] for i in range(3)], dtype=float)
            # ① inside frame?
            if not collision_checker.point_in_aabb(pt, self._boundary):
                return False
            # ② collision?
            for b in self._boxes:
                if collision_checker.point_in_aabb(pt, b):
                    return False
            return True

    world_bounds = np.asarray(self.boundary).ravel()[:6]
    obs_boxes    = np.asarray(self.blocks)[:, :6]
    si.setStateValidityChecker(_VC(si, world_bounds, obs_boxes))
    si.setup()

    # --- 3. Start / goal states ------------------------------------------
    def _to_state(arr):
        s = ob.State(space)
        for i in range(3): s[i] = float(arr[i])
        return s
    start_state = _to_state(start)
    goal_state  = _to_state(goal)

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start_state, goal_state, 0.1)

    # --- 4. Planner Factory ------------------------------------------
    _factory = {
        'RRT'        : og.RRT,
        'RRTConnect' : og.RRTConnect,
        'PRM'        : og.PRM,
    }
    if algo not in _factory:
        raise ValueError(f'Unsupported planner "{algo}"')

    planner = _factory[algo](si)
    for k, v in planner_kwargs.items():
        getattr(planner, f'set{k[0].upper()+k[1:]}')(v)
    planner.setProblemDefinition(pdef)
    planner.setup()

    # --- 5. Solve ------------------------------------------------------
    solved = planner.solve(runtime)
    if not solved:
        return np.array([start, goal])

    # optional simplify
    if simplify and hasattr(planner, 'getProblemDefinition'):
        og.PathSimplifier(si).simplifyMax(pdef.getSolutionPath())

    # --- 6. Convert OMPL path (string) -> NumPy array (N,3)------------------
    path = pdef.getSolutionPath().printAsMatrix()
    pts = np.fromstring(path, sep=' ').reshape(-1, 3)
    return pts
      

  def plan_greedy(self,start,goal):
    path = [start]
    numofdirs = 26
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        next = path[-1] + dR[:,k]
        
        # Check if this direction is valid
        if( next[0] < self.boundary[0,0] or next[0] > self.boundary[0,3] or \
            next[1] < self.boundary[0,1] or next[1] > self.boundary[0,4] or \
            next[2] < self.boundary[0,2] or next[2] > self.boundary[0,5] ):
          continue
        
        valid = True
        for k in range(self.blocks.shape[0]):
          if( next[0] >= self.blocks[k,0] and next[0] <= self.blocks[k,3] and\
              next[1] >= self.blocks[k,1] and next[1] <= self.blocks[k,4] and\
              next[2] >= self.blocks[k,2] and next[2] <= self.blocks[k,5] ):
            valid = False
            break
        if not valid:
          continue
        
        # Update next node
        disttogoal = sum((next - goal)**2)
        if( disttogoal < mindisttogoal):
          mindisttogoal = disttogoal
          node = next
      
      if node is None:
        break
      
      path.append(node)
      
      # Check if done
      if sum((path[-1]-goal)**2) <= 0.1:
        break
      
    return np.array(path)

