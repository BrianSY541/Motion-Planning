import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import Planner
import collision_checker

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))
  

def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks


def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h


def runtest(mapfile, start, goal, verbose = True):
  '''
  This function:
   * loads the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  MP = Planner.MyPlanner(boundary, blocks) # TODO: replace this with your own planner implementation
  
  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)

  # Call the motion planner
  t0 = tic()
  # path = MP.plan_search(start, goal)
  path = MP.plan_sample(start, goal,
                      algo='RRT',
                      runtime=1000000.0)
  toc(t0,"Planning")
  
  # Plot the path
  if verbose:
    ax.plot(path[:,0],path[:,1],path[:,2],'r-')

  # TODO: You should verify whether the path actually intersects any of the obstacles in continuous space
  # TODO: You can implement your own algorithm or use an existing library for segment and 
  #       axis-aligned bounding box (AABB) intersection
  collision_free = collision_checker.path_is_collision_free(
    waypoints=path,
    boxes=blocks[:, :6]                # shape (N,6)
  )
  
  goal_reached = bool(sum((path[-1]-goal)**2) <= 0.15)
  success = collision_free and goal_reached
  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))
  return success, pathlength


def test_single_cube(verbose = True):
  print('Running single cube test...\n') 
  start = np.array([7.0, 7.0, 5.5])
  goal = np.array([2.3, 2.3, 1.3])
  success, pathlength = runtest('./maps/single_cube.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')
  

def test_maze(verbose = True):
  print('Running maze test...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, pathlength = runtest('./maps/maze.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')

    
def test_window(verbose = True):
  print('Running window test...\n') 
  start = np.array([6, -4.9, 2.8])
  goal = np.array([2.0, 19.5, 5.5])
  success, pathlength = runtest('./maps/window.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')

  
def test_tower(verbose = True):
  print('Running tower test...\n') 
  start = np.array([4.0, 2.5, 19.5])
  goal = np.array([2.5, 4.0, 0.5])
  success, pathlength = runtest('./maps/tower.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')

     
def test_flappy_bird(verbose = True):
  print('Running flappy bird test...\n') 
  start = np.array([0.5, 4.5, 5.5])
  goal = np.array([19.5, 1.5, 1.5])
  success, pathlength = runtest('./maps/flappy_bird.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength) 
  print('\n')

  
def test_room(verbose = True):
  print('Running room test...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  success, pathlength = runtest('./maps/room.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')

def test_pillars(verbose = True):
  print('Running pillars test...\n') 
  start = np.array([0.5, 0.5, 0.5])
  goal = np.array([19, 19, 9])
  success, pathlength = runtest('./maps/pillars.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')


if __name__=="__main__":
  test_single_cube()
  test_maze()
  test_flappy_bird()
  test_pillars()
  test_window()
  test_tower()
  test_room()
  plt.show(block=True)






