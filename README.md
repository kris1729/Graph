
# Intoduction of Graph.
- It's a Non Linear Data Structure.

**Visualation** - It is very easy to visualation comp. to other data structre.

**RealLife-Use** -  Google-Map , linkdin , zomato , ola, Facebok and use for Data Analytics.

**Edges** That represent the realation b/w Vertex/ Nodes.

**Type** - Two Type Directed And UnDirected Graphs
 
 - in undirected graph only bidirection adge are present
 ### Acyclic graph

 ![](./Images/Acyclic.png)

### cyclic graph

![](./Images/cyclic.png)
### Directed acyclic graph

![](./Images/Dag.png)

### connected graph

![](./Images/connected.png)

### disconnected graph

![](./Images/disconnected.png)

### fully Connected graph
- total no of req. edges is  nC~2~  or n*(n-1)/2

![](./Images/fullConnected.png)

# Representation of Graph 
## 1. Using AdjMatrix 
Using 2D matrix use represent in which if any edge is present
in b/w the node(V->U) which is represent by adjMat[v][u]=1 and if not any edge is present than it is adj[v][u] = 0
it take **O(v2)** space and time complexity.

- > It store that information also that in between the node there is **no any edges**

 It take time to **add a new edge is O(1)**

 it take  time to **remove a edge is O(1)**

 it take  time to **seaarch a edge existence is O(1)**
 >**If graph is very dance(no of edges are maximum) then uses adjMat Rep**
 
 > But in the maximum cases uses adjList graph
### Code

```cpp
// take input no of vertex and edges
int vertex , edges;
cin>>vertex>>edges;
// init a 2D matrix of vertex*vertex
vector<vector<int>>adjMat(vertex,vector<int>(vertex,0));

int v , u ,wt;
for(int i =0;i<edges;i++){
    cin>>v>>u>>wt;
    // this is for the undirected graph
    adjMat[v][u] = wt;
    adjMat[u][v] = wt;
}


```

## 2. Using adjList 
- In which we init using vector of array or vector of list
for the weighted graph uses pair of list/ array for the representation.
- it take **O(V+2E) == O(V+E)  Time complexity**.
- it take **O(V+E) space complexity** in worst case when graph is 
  fully connected than it take **O(v2)** space.

It take time to **add a new edge is O(1)**

 it take  time to **remove a edge is O(N)** but using STL can be reduce in **O(1)**

 it take  time to **seaarch a edge existence is O(N)** but using STL can be reduce in O(1)
 >**When graph is Sparce (vertex has limited edge ) then uses adjList Rep** Ex. facebook 
 
 > It uses Maximum.
 

### if follow this type of syntex
```cpp
adjList[v].push_bacK(u);
adjList[u].push_back(v);
```



### code for undirected unweighted Graph
```cpp
// take input no of vertex and edges
int vertex,edges;
cin>>vertex>>edges;
vector<int>adhList[Vertex];

int u,v;
for(int i =0;i<edges;i++){

    cin>>u>>v;
    // for undirected graph
    adjList[v].push_bacK(u);
    adjList[u].push_back(v);
}

// for printing

for(int i =0;i<vertex;i++){
    cout<<i<<"->";
    for(int j =0;j<adj[i].size();i++)
    cout<<adj[i][j]<<" ";
    cout<<endl;
}
```
### Code for Undirected weighted graph
```cpp
int vertex,edges;
cin>>vertex>>edges;
vector<pair<int,int>>adjList[vertex];
int u ,v,wt;
for(int i =0;i<edges;i++){
    cin>>u>>v>>wt;
    adjList[u].push_back({v,wt});
    adjList[v].push_back(make_pair(u,wt));
}
// for printing same adjList[i][j].first 
```
---

# **Graph Traversing**

 two type of traversing , BFS & DFS
 
 ## 1.BFS
 its kowns as level order traversing. 
 ```cpp

vector<int>BFS(vector<int>adjList[],int V){
    vector<int>bfs;
    // make V size vecor to check node is visited or not
    vector<bool>visited(V,0);
    // start traversing form 0
    queue<int>q;
    q.push(0);
    //  mark it visited

    while(!q.empty()){
        int parent = q.front();
        // remove first element;
        q.pop();
        // store in ans vector
        bfs.push_back(parent);
        // traverse all child of this parent
        for(int child =0;child<adj[parent].size();child++){
            if(!visited[adjList[parent][child]]){
                visited[adjList[parent][child]]=1;
                q.push(adjList[parent][child]);
            }
        }
    }

}
 ```

## 2. DFS 
 - it will visit all unvisited neabhour one by one.

or
 - take one way , and go to the extrime position if last than return , if already visit than not take this path return at same path.
 ![](./Images/dfs.png)

time complexity O(V+2E)==O(V+E);
space complexity O(V)
 ### Recursion Code

 ```cpp

 void DFS(int node , vector<int>&adj[],vector<int>&visited , vector<int>&ans){
      visited[node]=1;
      ans.push_back(node);
      for(int i =0;i<adj[node].size();i++){
        if(!visited[adj[node][i]]){
            DFS(adj[node][i],adj , visited,ans);
        }
      }
 }

 ```
