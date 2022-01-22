Name: Anoosh Guddehithlu Prathap Kumar
UNCC ID: 801200789
Email ID: aguddehi@uncc.edu

Programming Language used: Python 3.5.2
Algorithm: Dijkstra’s Shortest Path Algorithm
Executable Code File: graph.py in command line.
Input Files: network.txt, queries.txt
Output File: output.txt


How to run the Project:
1. Open the command prompt.
2. Set the current directory to the location where the file is present.
3. Make sure that the file with graph details (network) and the query file are in the current working directory.
4. “Please write ‘quit’ at the end of file which consists of all the queries”.
5. At the command prompt enter:
python graph.py network.txt < queries.txt > output.txt

The output file will be stored as output.txt


Program Description:
Successfully implemented Open Shortest Path First using Dijkstra's Algorithm 
Also, implemented the priority queue using min-heap with an Efficient running time.


Data Structure Implementation:
Implementation of Priority Queue using min heap made program more efficient. 
The min-heap stores the distance from source vertex to destination vertex and extracts the nearest vertex very efficiently.

Classes Implemented:
The Vertex class which consists information of the vertex such as cost, 
vertex name, parent information, vertex status and maintains an 
adjacency list. 
The Edge Class consists edges in the graph including the source, 
destination, cost and status.
The Graph Class creates a graph by constructing an adjacency list which includes the information of the graph.
As we seen in the code the time complexity of the Dijkstra’s algorithm is O((|V|+|E|)lnV
The PriorityQueue Class holds the information of the min-heap data structure.


Finding Reachable vertices using the function: Reachable:
The concept of Breadth first search (BFS) algorithm is used here to find out the vertices that are reachable from each vertex to other in the graph.
This prints the Reachable vertices in Up status from each vertex whose status is Up.
Wkt running time complexity of BFS is O(V+E) for each vertex or edge that is up. However, in the worst case all Vertices and Edges will be up which makes total running time as O(V*(V + E)) as there are loops to be run for every vertex 


Changes to the Graph:
•	Add_Edge tailvertex headvertex transmit time: add a new edge or update a previous edge.
•	Delete_Edge tailvertex headvertex: delete an existing edge going from tail vertex to headvertex.
•	Vertex_Up Vertex: make the vertex status as up.
•	Vertex_Down Vertex: make the vertex as down.
•	Edge_Up tailvertex headvertex: make the edge status as up.
•	Edge_Down tailvertex headvertex: make the edge status as down.


Print the graph:
•    print
Reachable vertices:
•    Reachable: All Reachable vertices are returned.
Finding the Shortest Path:
•    path from_Vertex to_Vertex
Exit the program: 	
•    quit



