# Dijkstra's Shortest Path Algorithm 
# NAME: Anoosh Guddehithlu Prathap Kumar
# Student ID: 801200789

import sys
from queue import *
from sys import argv

# Constructing a Priority Queue and implementing min binary heap.
class Priority_Queue(object):
    def __init__(self):
        self.Vertices_List = [] 
    

    # Finding the Parent Node using function:
    def parentNode(self, i):
        if i % 2 == 0 and i != 0:
            return int((i/2)-1)
        elif i % 2 != 0:
            return int(i/2)
        else:
            return 0
    

    # Finding Right Element using function:
    def leftElement(self, i):
        return (2*i)+1

    
    # Finding Right Element using function:
    def rightElement(self, i):
        return (2*i)+2


    # Implementing Min_Heapify using function:
    def Min_Heapify(self, A, i, n):
    # A,i and n are array list, index and number of elements respectively where the Heapify operation takes place.
        le = self.leftElement(i)
        re = self.rightElement(i)
        if le <= n and A[le][0] < A[i][0]:
            Smallest = le
        else:
            Smallest = i
        if re <= n and A[re][0] < A[Smallest][0]:
            Smallest = re
        if smallest != i:
            A[i], A[Smallest] = A[Smallest], A[i]
            self.Min_Heapify(A, Smallest, n)

    # Building min heap using function.
    def build_minHeap(self, A, n):
        for i in range(int(n/2)-1, -1, -1):
            self.Min_Heapify(A, i, n-1)

    # Adding/inserting elements to the priority queue.
    def add(self, A, ele):
        A.insert(len(A), ele)
        self.build_minHeap(A, len(A))

    # Decreasing priority queue. 
    def Decrease_Priority(self, A, i, key):
        A[i] = key
        while i > 0 and A[self.parentNode(i)][0] > A[i][0]:
            A[i], A[self.parentNode(i)] = A[self.parentNode(i)], A[i]
            i = self.parentNode(i)

    # Extracting least element using function.
    def extract_min(self, A):
        mini = A[0]
        A[0] = A[len(A)-1]
        del A[len(A)-1]
        self.Min_Heapify(A, 0, (len(A) - 1))
        return mini[1]

# Class called Vertex is created to store info of the vertices respectively.
class Vertex(object):
    def __init__(self, Vertex_Name, status_value):
        self.name = Vertex_Name     
        self.status_value = status_value        
        self.parentNode = None          
        self.cost_value = float('inf')    
        self.adj = []               
        #status_value is used to tell the status of the vertex.
        
    # Reset function is used to reset the info. 
    def Reset(self):                
        self.distance = float('inf')
        self.previous = None
        

# Class called Edge is created to store info of the Edges respectively.
class Edge(object):
    def __init__(self, Source_Vertex, Destination_Vertex, cost_value, status_value):
        self.Source_Vertex = Source_Vertex              # starting vertex 
        self.Destination_Vertex = Destination_Vertex    # final vertex 
        self.status_value = status_value                           
        self.cost_value = float(cost_value)             # cost_value is the weights of the edges
        
# Class called Graph is created to construct a graph.        
class Graph(object):
    def __init__(self):
        self.vertices = {}          # vertices list.
        self.edges = {}             # list of edges.
        self.Adj_list = {}           # adjacency list of the vertices.

    # Adding vertices to the graph.
    def Add_Vertex(self, name, Vertex):
        self.vertices[name] = Vertex

    # Setting Vertex status to Down.    
    def Vertex_Down(self, Vertex):
        self.vertices[Vertex].status_value = False

    # Setting Vertex status to UP.
    def Vertex_Up(self, Vertex):
        self.vertices[Vertex].status_value = True
    
    # Adding edges to the graph.  
    def Add_Edge(self, va_to_vb, edge):        
        if va_to_vb[0] not in self.vertices:          # if source Vertex is not available.
            self.Add_Vertex(va_to_vb[0], Vertex(va_to_vb[0], True))  
        
        if va_to_vb[1] not in self.vertices:          # if destination Vertex is not available.
            self.Add_Vertex(va_to_vb[1], Vertex(va_to_vb[1], True))
        
        if (va_to_vb[0], va_to_vb[1]) in self.edges:  # updating weights of edges.
            self.edges[(va_to_vb[0],va_to_vb[1  ])].cost_value = float(edge.cost_value)
        else:
            self.Add_AdjVertex(va_to_vb[0], va_to_vb[1])
            self.Add_AdjVertex(va_to_vb[1], None)
            self.edges[va_to_vb] = edge
    
    # Adding Vertices to the list.
    def Add_AdjVertex(self, va, vb):
        if vb == None:
            self.Adj_list.setdefault(va,[])
        else:
            self.Adj_list.setdefault(va,[]).append(vb)    
    
    # Setting Edge status to Down.
    def Edge_Down(self, va, vb):
        self.edges[(va, vb)].status_value = False

    # Setting Edge status to Up.
    def Edge_Up(self, va, vb):
        self.edges[va, vb].status_value = True

    # Delete an edge between va and vb.
    def Delete_Edge(self, va, vb):
        del self.edges[(va,vb)]
        self.Adj_list[va].remove(vb)

    # Outputting the Graph.
    def Print_Graph(self):
        for ve in sorted(self.vertices.keys()):
            if self.vertices[ve].status_value == False:
                print(self.vertices[ve].name, "DOWN")
            else:
                print(self.vertices[ve].name) 

            for Adj_Vertex in sorted(self.Adj_list[ve]):
                if (self.edges[(ve,Adj_Vertex)].status_value == False): 
                    print(" ", Adj_Vertex,self.edges[(ve,Adj_Vertex)].cost_value, "DOWN")  
                else:
                    print(" ", Adj_Vertex,self.edges[(ve,Adj_Vertex)].cost_value)  
    
    # Printing vertices that is rechable from each vertex.
    def Print_Reachable(self):
        for Vertex in (sorted(self.vertices.keys())):
            if self.vertices[Vertex].status_value == True:
                self.Reachable(Vertex)

    # Breadth First Search algorithm is implemented to reach all vertices which actually takes O(V+E) time, but may take O(V*(V + E)) time in worst case.
    def Reachable(self, Vertex):
        Discovered_Vertices = {}
        Reachable_Vertices = {}
        for ver in self.Vertices.keys():
            Discovered_Vertices[ver] = "white" 
        Discovered_Vertices[vertex] = "gray"        
        queue = Queue()
        queue.put(Vertex)
        while not queue.empty():
            get = queue.get()
            for ve in sorted(self.Adj_list[get]):
                if Discovered_Vertices[ve] == "white" and self.vertices[ve].status_value == True and self.edges[(get,ve)].status_value == True:
                    Discovered_Vertices[ve] == "gray"
                    queue.put(ve)
                    Reachable_Vertices[ve] = ve
            Discovered_Vertices[get] = "black"
        print(Vertex)
        for vert in sorted(Reachable_Vertices):
            print(" ", vert)
        
    # Reset function.
    def Clear_All(self):
        for Vertex in self.vertices.values():
            Vertex.Reset()

    # Finding Shotest_Path from source vertex to destination vertex
    # Using Dijsktra's Shortest Path Algorithm which uses time O((|V|+|E|)lnV).
    def Shortest_Path(self, Source, Destination):
        P_Q = Priority_Queue()
        for Vertex in self.vertices.keys():
            self.vertices[Vertex].parentNode = None
            self.vertices[Vertex].cost_value = float('inf')
        self.vertices[Source].cost_value = 0.0
        distance = []
        for weight in self.vertices:
            distance.insert(len(distance), (self.vertices[weight].cost_value, self.vertices[weight]))
        P_Q.build_minHeap(distance, len(distance))  # Constructing Binary Hin Heap 
        S1 = []
        while distance:
            ve = P_Q.extract_min(distance)  # extract min extracts the minimum from queue
            if ve.status_value == False:  
                continue
            else:
                S1.insert(len(S1), ve)  # Visited Vertex is marked.
                for ele in self.Adj_list[ve.name]:  
                    if self.vertices[ele].status_value == True and self.edges[(ve.name, ele)].status_value == True:
                        Previous_Distance = self.vertices[ele].cost_value
                        if self.vertices[ele].cost_value > (self.vertices[ve.name].cost_value + self.edges[(ve.name,ele)].cost_value) :
                            self.vertices[ele].cost_value = self.vertices[ve.name].cost_value + self.edges[(ve.name,ele)].cost_value 
                            self.vertices[ele].parentNode = ve
                            index = distance.index((Previous_Distance, self.vertices[ele]))
                            P_Q.Decrease_Priority(distance, index, (self.vertices[ele].cost_value, self.vertices[ele]))
        node = self.vertices[Destination]
        while node.parentNode is not None:   #Display the vertices in the correct order
            distance.append(node.name)
            node = node.parentNode
        distance.append(node.name)
        distance.reverse()
        print(" ".join([str(vert) for vert in distance]),"%.2f" % self.vertices[Destination].cost_value)


# Function below takes input file from command line, open, reads and stores.
def main():
    Input_File = argv[1]
    f = open(Input_File,'r')
    graph = Graph()
    for line in f:
        node = line.split()
        if len(node) != 3:
            print("Formatted_Line ", end="")
            print(node)
            exit()
        else:
            va = Vertex(line.split()[0], True)  # Vertex a
            vb = Vertex(line.split()[1], True)  # Vertex b
            graph.Add_Vertex(va.name,va)         # Adding Vertex a
            graph.Add_Vertex(vb.name,vb)         # Adding vertex b
            Edge_a = Edge(line.split()[0], line.split()[1], line.split()[2], True)   # Edge a    
            Edge_b = Edge(line.split()[1], line.split()[0], line.split()[2], True)   # Edge b
            graph.Add_Edge((va.name, vb.name), Edge_a)    # Adding Edge a
            graph.Add_Edge((vb.name, va.name), Edge_b)    # Adding Edge b
    
    f.close()   

    # In this part each query will be read.
    while True:
        line = sys.stdin.readline()
        if line.strip():
            Input_Query = line.split()
            if len(Input_Query) == 4:
                if Input_Query[0] == "Add_Edge":
                    ea = Edge(Input_Query[1], Input_Query[2], Input_Query[3], True)
                    graph.Add_Edge((Input_Query[1],Input_Query[2]), ea)
                else:
                    print("Wrong command, try again.")

            elif len(Input_Query) == 3:
                if Input_Query[0] == "Delete_Edge":
                    graph.Delete_Edge(Input_Query[1], Input_Query[2])

                elif Input_Query[0] == "Edge_Down":
                    graph.Edge_Down(Input_Query[1], Input_Query[2])

                elif Input_Query[0] == "Edge_Up":
                    graph.Edge_Up(Input_Query[1], Input_Query[2])

                elif Input_Query[0] == "Shortest_Path":
                    if Input_Query[1] not in graph.vertices.keys():
                        print("Could not find the Source Vertex.")
                    
                    elif Input_Query[2] not in graph.vertices.keys():
                        print("Could not find the Destination Vertex.")
                    
                    else:
                        graph.Shortest_path(Input_Query[1],Input_Query[2])

                else:
                    print("Wrong Command, try again.")

            elif len(Input_Query) == 2:
                if Input_Query[0] == "Vertex_Down":
                    graph.Vertex_Down(Input_Query[1])

                elif Input_Query[0] == "Vertex_Up":
                    graph.Vertex_Up(Input_Query[1])

                else:
                    print("Wrong Command, try again.")
                
            elif len(Input_Query) == 1:
                if Input_Query[0] == "Reachable":
                    graph.Print_Reachable()

                elif Input_Query[0] == "print":
                    graph.Print_Graph()
                
                elif Input_Query[0].lower() == "quit":
                    break

                else:
                    print("Wrong Command. Please try again")
                
            else:
                print("Wrong Command. Please try again")

# calling the main function:
if __name__ == '__main__':
    main()