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
```

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
- dfs code can right using stack , 
> which code can be write using recursion ,
this can be also write using stack also

```cpp

vector<int>dfs(vector<int>adj[],int V){
    vecotor<int>ans;
    // make a stack 
    stack<int>st;
    // make a visit vector
    vecotr<int>vis;
    // traversing start form the 0;
    st.push(0);
    while(!st.empty()){
        int node = st.top();
        st.pop();
        if(!vis[node]){
            // make visited
            vis[node]=1;
            ans.push_back(node);
            // ans is read left to riight
            for(int i = adj[node].size();i>=0;i--){
                  st.push(adj[node][i]);
            }
        }
    }
    return  ans;
}

```
--- 

# Cycle Dedection in a Graph
## 1. using DFS 
- Logic , if we vist a node which is already visited then we  can sey that in which a cycle is present, but but but 
it can give worng ans , when it vist it's parent at same time .
**Change logic** we apply dfs but current node will not visit it's parent node.

### code
```cpp
bool solve(vector<int>adj[],vector<bool>&vis,int node , int parent){
   vis[node]=1;
        for(int i =0;i<adj[node].size();i++){
            // when next node is parent node
            if(adj[node][i]==parent)
             continue;
            // when next node is already visted
            if(vis[adj[node][i]])
              return 1;
            // next node is not visted 
            if(solve(adj,vis,adj[node][i],node))
             return 1;
        }
        return false;
}
    
    bool isCycle(int V, vector<int> adj[]) {
        
      vector<bool>vis(V,0);
      return solve(adj , vis , i , -1);
    }

```
**Note this code give worng ans**

this code is only work for **Connected** graph not for disconnected.
so we modefiy the code 
- we will call the function for every node if any node give true ans
then in this graph cycle will present or not

### Correct code
```cpp
ool solve(vector<int>adj[],vector<bool>&vis,int node , int parent){
   vis[node]=1;
        for(int i =0;i<adj[node].size();i++){
            if(adj[node][i]==parent)
             continue;
            if(vis[adj[node][i]])return 1;
            
            if(solve(adj,vis,adj[node][i],node))
            return 1;
        }
        return false;
}
    
    bool isCycle(int V, vector<int> adj[]) {
        
      vector<bool>vis(V,0);
      for(int i =0;i<V;i++){
          if(!vis[i]&&solve(adj , vis , i , -1))
           return 1;
    }
    return false;}
```
time complexity = O(v+E);

---

<!-- date - 16-08-2024 -->

> # Graph and Vertices
![](./Images/Question/no%20of%20vertex.png)

```cpp
 long long count(int n) {
        long long ans  = pow(2,(n*(n-1)/2));
        return ans;
    }
```

> # Print adjacency list
![](./Images/Question/Print%20adjacency%20list.png)

```cpp
 vector<vector<int>> printGraph(int V, vector<pair<int,int>>edges) {
       vector<vector<int>>ans(V);
       for(auto x : edges)
       {
          ans[x.first].push_back(x.second);
          ans[x.second].push_back(x.first);
       }
       return ans;
    }
```

> # BFS of graph
![](./Images/Question/BFS%20of%20graph.png)

```cpp
vector<int> bfsOfGraph(int V, vector<int> adj[]) {
        // here is given starting node 0
        vector<bool> vis(V, 0);
        queue<int> q;
        q.push(0);
        vis[0] = true;
        vector<int> ans;
        while (!q.empty()) {
            int node = q.front();
            q.pop();
            ans.push_back(node);
            for (auto x : adj[node]) {
                if (!vis[x]) {
                    q.push(x);
                    vis[x] = true;
                }
            }
        }
        return ans;
    }
```
> # DFS of Graph
![](./Images/Question/DFS%20of%20Graph.png)

```cpp
void dfs(int node , vector<int>&ans, vector<bool>&vis, vector<int>adj[]){
      vis[node] = true;
      ans.push_back(node);
      for(auto x : adj[node]){
          if(!vis[x])dfs(x,ans,vis,adj);
      }
  }
  
    vector<int> dfsOfGraph(int V, vector<int> adj[]) {
        vector<int>ans;
        vector<bool>vis(V,0);

        dfs(0,ans,vis,adj);
        return ans;
    }

```

stack using code 

```cpp
 vector<int> dfsOfGraph(int V, vector<int> adj[]) {
       vector<bool>vis(V,0);
       vector<int>ans;
       
       stack<int>st;
       st.push(0);
       while(!st.empty()){
           int node = st.top();
            st.pop();
           if(!vis[node]){
           ans.push_back(node);
           vis[node]=1;
          
           for(int i= adj[node].size()-1 ; i>=0;i--)
               st.push(adj[node][i]);
           } 
       }
       return ans;
    }
```

> # Number of Provinces
![](./Images/Question/Number%20of%20Provinces.png)

```cpp
 // code of dfs
    void dfs(int node, vector<bool>& vis, vector<int> adj[]) {
        vis[node] = true;
        for (auto x : adj[node]) {
            if (!vis[x])
                dfs(x, vis, adj);
        }
    }
// main code
    int findCircleNum(vector<vector<int>>& isConnected) {
        int n = isConnected.size();
        // convert matrix in vector of list
        vector<int> adj[n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (isConnected[i][j] == 1)
                    adj[i].push_back(j), adj[j].push_back(i);

        // call for dfs and increase the ans
        int cnt = 0;
        vector<bool> vis(n, 0);
        for (int i = 0; i < n; i++) {
            if (!vis[i]) {
                dfs(i, vis, adj);
                cnt++;
            }
        }
        return cnt;
    }
```
> # Number of Provinces
![](./Images/Question/Number%20of%20Provinces-2.png)
using bfs and adj list
```cpp
// function for bfs
    void bfs(int node, vector<bool>& vis, vector<int> adj[]) {
        queue<int> q;
        q.push(node);
        vis[node] = true;
        while (!q.empty()) {
            int nod = q.front();
            q.pop();
            for (auto x : adj[nod])
                if (!vis[x]) {
                    vis[x] = true;
                    q.push(x);
                }
        }
    }
    // main function
    int numProvinces(vector<vector<int>> arr, int V) {
        vector<int> adj[V];
        // make the adj list
        for (int i = 0; i < V; i++)
            for (int j = 0; j < V; j++)
                if (arr[i][j] == 1 && i != j) {
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                }

        // solve using bfs
        vector<bool> vis(V, 0);
        int cnt = 0;
        for (int i = 0; i < V; i++)
            if (!vis[i]) {
                bfs(i, vis, adj);
                cnt++;
            }
        return cnt;
    }
```
> # Rotting Oranges
![](./Images/Question/otting%20Oranges.png)


```cpp
// main function
int orangesRotting(vector<vector<int>>& grid) {
        queue<pair<pair<int, int>, int>> q;
        int n = grid.size(), m = grid[0].size();
        vector<vector<bool>> vis(n, vector<bool>(m, 0));
        // push rotted orenge
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                if (grid[i][j] == 2) {
                    vis[i][j] = true;
                    q.push({{i, j}, 0});
                }
                // bfs code
        int tm = 0;
        while (!q.empty()) {
            int r = q.front().first.first;
            int c = q.front().first.second;
            int t = q.front().second;
            q.pop();
            grid[r][c] = 2;
            tm = max(tm, t);
            vector<int> drow = {1, -1, 0, 0}, dcol = {0, 0, -1, 1};
            for (int i = 0; i < 4; i++) {
                int row = drow[i] + r;
                int col = dcol[i] + c;
                if (row >= 0 && col >= 0 && row < n && col < m &&
                    !vis[row][col] && grid[row][col] == 1) {
                    q.push({{row, col}, t + 1});
                    vis[row][col] = true;
                }
            }
        }
        // check for any on is fress
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                if (grid[i][j] == 1 && !vis[i][j])
                    return -1;
            // totale time
        return tm;
    }
```
> # Flood Fill
![](./Images/Question/Flood%20Fill.png)
```cpp
   void solve(int r , int c , vector<vector<int>>& ans , int color , int inColor , vector<int>&nrow , vector<int>ncol ){
    ans[r][c] = color;
    int n = ans.size();
    int m = ans[0].size();
    for(int i =0;i<4;i++){
       int row = r + nrow[i];
       int col = c + ncol[i];
       if(row>=0&&row<n && col>=0&&col<m && ans[row][col]==inColor && ans[row][col]!=color)
       solve(row , col, ans , color , inColor , nrow , ncol);
    }
   }
    vector<vector<int>> floodFill(vector<vector<int>>& image, int sr, int sc, int color) {
        vector<vector<int>>ans = image;
        int inColor = ans[sr][sc];
        // optional
        if(inColor==color)return ans;
        vector<int> nrow = {-1,0,1,0};
        vector<int> ncol = {0,1,0,-1};
        solve(sr,sc,ans,  color,inColor,nrow ,ncol);
        return ans;
    }
```
> # Undirected Graph Cycle
![](./Images/Question/Undirected%20Graph%20Cycle.png)
code using bfs
```cpp
 bool bfs(int node , vector<bool>&vis, vector<int>adj[]){
        queue<pair<int,int>>q;
        q.push({node,-1});
        vis[node] = true;
        
        while(!q.empty()){
            int nod = q.front().first;
            int parent = q.front().second;
            q.pop();
            for(auto x : adj[nod]){
                if(x==parent)continue;
                if(vis[x])return true;
                q.push({x,nod});
                vis[x] = true;
            }
           
        }
        return false;
    }
    bool isCycle(int V, vector<int> adj[]) {
       // code using bfs;
       vector<bool>vis(V,0);
       for(int i =0;i<V;i++){
        if(!vis[i]&&bfs(i,vis,adj)==true)return true;}
        
        return false;
    }
```
code using dfs
```cpp
// dfs function
bool dfs(int node, vector<bool>& vis, vector<int> adj[], int parent) {
        vis[node] = true;
        for (auto child : adj[node]) {
            if (child == parent)continue;
            if (vis[child]) return true;
            if (dfs(child, vis, adj, node) == true)return true;
        }
        return false;
    }
    // main code
    bool isCycle(int V, vector<int> adj[]) {

        vector<bool> vis(V, 0);
        for (int i = 0; i < V; i++)
            if (!vis[i] && dfs(i, vis, adj, -1))
                return true;
        return false;
    }
```

> # 01 Matrix  ## Important
![](./Images/Question/01%20Matrix.png)

approch --> for all zero the step will be 0 than push all of them and after this run bfs all all zero with stemp++

```cpp
   vector<vector<int>> updateMatrix(vector<vector<int>>& mat) {
        // make visited vector
        int n = mat.size(), m = mat[0].size();
        vector<vector<bool>> vis(n, vector<bool>(m, 0));
        // make ans vector
        vector<vector<int>> ans(n, vector<int>(m, -1));
        // make a q for bfs ({{row,col},step})
        queue<pair<pair<int, int>, int>> q;
        // insert all Zero with zero step

        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                if (mat[i][j] == 0) {
                    vis[i][j] = true;
                    q.push({{i, j}, 0});
                }

        // apply bfs all the zero
        vector<int> drow = {-1, 1, 0, 0};
        vector<int> dcol = {0, 0, -1, 1};
        while (!q.empty()) {
            int r = q.front().first.first;
            int c = q.front().first.second;
            int step = q.front().second;
            q.pop();
            // update the ans
            ans[r][c] = step;

            for (int i = 0; i < 4; i++) {
                int row = r + drow[i];
                int col = c + dcol[i];
                if (row >= 0 && col >= 0 && row < n && col < m &&
                    !vis[row][col]) {
                    vis[row][col] = true;
                    update step to step+1
                    q.push({{row, col}, step + 1});
                }
            }
        }
        return ans;
    }
```