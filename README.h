/*
This file is for explanation only 

2.Head files:
  2.1 Global: including global varibles, adjacency matrix for Dijkstra shortest path algorithm
  2.2 Offset: including offset parameter for different functions - allows motors to have a slight variation

To change navigation matrix (i.e the board used), first type out the new adjacency matrix into the Nav_matrix variable following the conventipon outlined in documentation
Then reset all the array length definitions in navigate and ensure they are correctly defined with initial values (false in visited etc)
Once this is done edit the function IDP with the new pick up/drop off locations and the desired number of boxes to pick up

The procedure for Dijkstra is:

From start node update the distances to each connected node and store the previous node (i.e. the start node) as the previously visited node
Now set your current node to the closest non visited node and repeat the process, updating any routes that are shorter than the previous stored closest route
Repeat this until you have "visited every node"
You now have stored the length of the closest distance between nodes as well as the previous node on that shortest path - now work backwards from the destination and this will give the sequence of nodes

*/
