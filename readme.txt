Implemented features:

- Automatic initialization of the boids. All boids should be randomly initialized within the initialization radius with a random forward heading within the forward random range.

- Handling of the resetting of values at the top of the simulation loop

- Implementation of building the neighbour list, by simulating vision using distance and dot product.

- Implementation of separation rule, alignment rule, cohesion rule, no neighbour wander rule, obstacles rule

- Addition of the world boundary to the obstacles rule

- The total forces are accumulated

-  SetGoal is implemented, implementing the behaviour described above. A target is set, path calculated.

- boidzero is handled. A check is made to ensure we should be processing the path (navigating + ready + enough corners). Position is correctly sampled from the navmesh. Bookkeeping for corners is correct and all corners are followed. Finishing the path is handled cleanly and variables are reset as needed.

- The boid objects (animated meshes) are updated and follow the path and heading of the boid particles.

- The symplectic Euler integration scheme is implemented.

- The simulator loop updates all boid states using the update callback and time. 

- The recorded testcase requires that the default values in the testcase are preserved.