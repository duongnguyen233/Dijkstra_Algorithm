#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#define BUFFER_STR 256

using namespace std;

//declare the global variables
int fileLineNumber = 122;
int numVertices = 0;
int numEdges = 0;
int startVertice = 0;
int endVertice = 0;

int** graph;

struct Vertex {
    int vertexLabel = 0;
    double xCoor = 0.00;
    double yCoor = 0.00;
};

struct Edge {
    int startVertex = 0;
    int endVertex = 0;
    double weight = 0.00;
};


//implement Linked List
template<typename T>
struct node
{
    int index;
    T data; // data
    node* next;
};

template<typename T>
class linked_list
{
private:
    node<T>* head, * tail;
    int currIndex = 0;
    int size = 0;
public:

    linked_list();

    int getSize();
    void add_node(T data);
    node<T>* getNodeIdx(int idx);
};

template<typename T>
int linked_list<T>::getSize() {
    return this->size;
}

template<typename T>
linked_list<T>::linked_list() {
    head = NULL;
    tail = NULL;
}

template<typename T>
void linked_list<T>::add_node(T data)
{
    node<T>* tmp = new node<T>();
    tmp->data = data; // asign data
    tmp->next = NULL;
    tmp->index = size;

    if (head == NULL)
    {
        head = tmp;
        tail = tmp;
    }
    else
    {
        tail->next = tmp;
        tail = tail->next;
    }
    size++;
}

template<typename T>
node<T>* linked_list<T>::getNodeIdx(int idx) {
    node<T>* ret = NULL;
    node<T>* ptr;
    ptr = head;
    while (ptr != NULL) {
        if (ptr->index == idx) {
            ret = ptr;
            break;
        }
        ptr = ptr->next;
    }
    return ret;
}

linked_list<Vertex> vertexList;
linked_list<Edge> edgeList;

double calcDistance() {
    double x1 = 0;
    double x2 = 0;
    double y1 = 0;
    double y2 = 0;

    for (int i = 0; i < vertexList.getSize(); i++) {
        if (vertexList.getNodeIdx(i)->data.vertexLabel == startVertice) {
            x1 = vertexList.getNodeIdx(i)->data.xCoor;
            y1 = vertexList.getNodeIdx(i)->data.yCoor;
        }
        if (vertexList.getNodeIdx(i)->data.vertexLabel == endVertice) {
            x2 = vertexList.getNodeIdx(i)->data.xCoor;
            y2 = vertexList.getNodeIdx(i)->data.yCoor;
        }
    }
    //cout << x1 << "\t" << x2 << "\t" << y1 << "\t" << y2;
    double x = x1 - x2; //calculating number to square in next step
    double y = y1 - y2;
    double dist = 0;

    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    return dist;
}

int minDistance(int dist[], bool sptSet[])
{
    // Initialize min value
    int min = INT_MAX, min_index;

    for (int v = 0; v < numVertices; v++) {
        if (sptSet[v] == false && dist[v] <= min) {
            min = dist[v], min_index = v;
        }
    }

    return min_index;
}

void printResult(int dist[], int src, int dest)
{
    // cout << "StartVertex \t StartVertex \t Distance" << endl;
    // cout << (src + 1) << "\t\t" << (dest + 1) << "\t\t" << dist[dest] << endl;
    cout << "The length of the shortest path: " << dist[dest] << endl;
}

void dijkstraAlgorithmMin(int src, int dest) {
    int* dist = new int[numVertices]; // The output array.  dist[i] will hold the
    // shortest
    // distance from src to i

    bool* sptSet = new bool[numVertices]; // sptSet[i] will be true if vertex i is
    // included in shortest
    // path tree or shortest distance from src to i is
    // finalized
 
    // Initialize all distances as INFINITE and stpSet[] as
    // false
    int i = 0;
    for (i = 0; i < numVertices; i++) {
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }
 
    // Distance of source vertex from itself is always 0
    dist[src] = 0;
 
    // Find shortest path for all vertices
    for (int count = 0; count < numVertices - 1; count++) {
        // Pick the minimum distance vertex from the set of
        // vertices not yet processed. u is always equal to
        // src in the first iteration.
        int u = minDistance(dist, sptSet);
 
        // Mark the picked vertex as processed
        sptSet[u] = true;
 
        if (!sptSet[dest]
            && graph[u][dest]
            && (dist[u] != INT_MAX)
            && ((dist[u] + graph[u][dest]) < dist[dest])) {
                dist[dest] = dist[u] + graph[u][dest];
                // cout << "(" << (u + 1) << "," << (dest + 1) << ")-->";
                cout << "Shortest path: " << (u + 1) << " -> " << (dest + 1) << endl;
        }
    }

    printResult(dist, src, dest);
    delete[] dist;
    delete[] sptSet;
}

void printResult() {
    cout << "==========================================================================================================================" << endl;
    cout << "The number of vertexes in the graph: " << numVertices << endl;
    cout << "The number of edges in the graph: " << numEdges << endl;
    cout << "The start vertex: " << startVertice << endl;
    cout << "The end vertex: " << endVertice << endl;
    cout << "==========================================================================================================================" << endl;
    cout << "The Euclidean distance between the start and the goal vertexes: " << calcDistance() << endl;
}

void readDataFromFile() {
     char filename[100];
    ifstream inputFile;
    cout << "Please enter the file name: ";
    cin >> filename;
    inputFile.open(filename);
    if (!inputFile) {
        cout << "Open the file failed.";
    }
    
    if (inputFile.is_open()) {
        char buffer[BUFFER_STR];
        int lineIdx = 1;
        while (inputFile.getline(buffer, sizeof(buffer))) {
            if ((lineIdx == 1) || (lineIdx == fileLineNumber)) {
                char strTemp[2][BUFFER_STR];
                memset(strTemp, '\0', sizeof(strTemp));
                int idx = 0;
                int cnt = 0;
                for (int i = 0; i < BUFFER_STR; i++) {
                    if (buffer[i] == '\0') {
                        break;
                    }
                    if (buffer[i] != '\t') {
                        strTemp[idx][cnt] = buffer[i];
                        cnt++;
                    }
                    else {
                        if (idx == 0) {
                            if (lineIdx == fileLineNumber) {
                                startVertice = atoi(strTemp[idx]);
                            }
                            else {
                                numVertices = atoi(strTemp[idx]);
                            }
                            cnt = 0;
                        }
                        else if (idx == 1) {
                            if (lineIdx == fileLineNumber) {
                                endVertice = atoi(strTemp[idx]);
                            }
                            else {
                                numEdges = atoi(strTemp[idx]);
                            }
                            cnt = 0;
                        }
                        idx++;
                    }
                }
                /*std::cout << test1[0] << "\n";*/
            }
            else if ((lineIdx > 1) && (lineIdx < numVertices + 2)) {
                char strTemp[3][BUFFER_STR];
                memset(strTemp, '\0', sizeof(strTemp));
                Vertex tempVer;
                int idx = 0;
                int cnt = 0;
                for (int i = 0; i < BUFFER_STR; i++) {
                    if ((buffer[i] != '\t') && (buffer[i] != '\0')) {
                        strTemp[idx][cnt] = buffer[i];
                        cnt++;
                    }
                    else {
                        if (idx == 0) {
                            tempVer.vertexLabel = atoi(strTemp[idx]);
                            cnt = 0;
                        }
                        else if (idx == 1) {
                            tempVer.xCoor = atoi(strTemp[idx]);
                            cnt = 0;
                        }
                        else {
                            tempVer.yCoor = atoi(strTemp[idx]);
                            cnt = 0;
                            vertexList.add_node(tempVer);
                            break;
                        }
                        idx++;
                    }
                }
            }
            else {
                char strTemp[3][BUFFER_STR];
                memset(strTemp, '\0', sizeof(strTemp));
                Edge tempEdg;
                int idx = 0;
                int cnt = 0;
                for (int i = 0; i < BUFFER_STR; i++) {
                    if ((buffer[i] != '\t') && (buffer[i] != '\0')) {
                        strTemp[idx][cnt] = buffer[i];
                        cnt++;
                    }
                    else {
                        if (idx == 0) {
                            tempEdg.startVertex = atoi(strTemp[idx]);
                            cnt = 0;
                        }
                        else if (idx == 1) {
                            tempEdg.endVertex = atoi(strTemp[idx]);
                            cnt = 0;
                        }
                        else {
                            tempEdg.weight = atoi(strTemp[idx]);
                            cnt = 0;
                            edgeList.add_node(tempEdg);
                            break;
                        }
                        idx++;
                    }
                }
            }
            lineIdx++;
        }
    }
}

class Graph {
public:
    int V; // Number of vertices
    vector<vector<pair<int, int>> > adj; // Adjacency list with (vertex, weight) pairs

    Graph(int vertices) : V(vertices) {
        adj.resize(V);
    }

    void addEdge(int u, int v, int weight) {
        adj[u].emplace_back(v, weight);
    }

    // Recursive function to find the longest path with distance
    void longestPathUtil(int u, int destination, vector<int>& currentPath, vector<int>& longestPath, vector<bool>& visited, int& maxDistance, int currentDistance) {
        visited[u] = true;
        currentPath.push_back(u);

        if (u == destination) {
            if (currentDistance > maxDistance) {
                maxDistance = currentDistance;
                longestPath = currentPath;
            }
        }
        else {
            for (const auto& edge : adj[u]) {
                int v = edge.first;
                int weight = edge.second;
                if (!visited[v]) {
                    longestPathUtil(v, destination, currentPath, longestPath, visited, maxDistance, currentDistance + weight);
                }
            }
        }
        visited[u] = false;
        currentPath.pop_back();
    }

    // Function to find the longest path from source to destination with distance
    pair<vector<int>, int> longestPathWithDistance(int source, int destination) {
        vector<bool> visited(V, false);
        vector<int> currentPath, longestPath;
        int maxDistance = INT_MIN;

        longestPathUtil(source, destination, currentPath, longestPath, visited, maxDistance, 0);

        return { longestPath, maxDistance };
    }
};

int main() {
    //read file
    readDataFromFile();

    //print result
    printResult();

    // Create matrix map
    // Declare memory block of size M
    graph = new int* [numVertices];
    for (int i = 0; i < numVertices; i++) {
        // Declare a memory block
        // of size n
        graph[i] = new int[numVertices];
    }

    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < numVertices; j++) {
            graph[i][j] = 0;
        }
    }

    for (int i = 0; i < edgeList.getSize(); i++) {
        int x = edgeList.getNodeIdx(i)->data.startVertex - 1;
        int y = edgeList.getNodeIdx(i)->data.endVertex - 1;
        graph[x][y] = edgeList.getNodeIdx(i)->data.weight;
        graph[y][x] = edgeList.getNodeIdx(i)->data.weight;
    }

    // Agorithm find min Path
    dijkstraAlgorithmMin(startVertice - 1, endVertice - 1);

    // Function call
    Graph g(numVertices);
    for (int i = 0; i < edgeList.getSize(); i++) {
        int x = edgeList.getNodeIdx(i)->data.startVertex - 1;
        int y = edgeList.getNodeIdx(i)->data.endVertex - 1;
        int w = edgeList.getNodeIdx(i)->data.weight;
        g.addEdge(x, y, w);
    }

    int source = startVertice - 1;
    int destination = endVertice - 1;

    pair<vector<int>, int> longestPathInfo = g.longestPathWithDistance(source, destination);
    vector<int> longestPath = longestPathInfo.first;
    int maxDistance = longestPathInfo.second;

    if (longestPath.empty()) {
        cout << "No path exists from " << source << " to " << destination << "." << endl;
    }
    else {
        cout << "Longest Path: ";
        for (int vertex : longestPath) {
            if(vertex != longestPath.back()){
                cout << (vertex + 1) << " -> ";
            }
            else 
                cout << (vertex + 1);
        }
        cout << endl << "The length of the longest path: " << maxDistance << endl;
        cout << "==========================================================================================================================" << endl;
    }

    delete[] graph;
    return 0;
}