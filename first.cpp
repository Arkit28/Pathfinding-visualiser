#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <limits>
#include <chrono>

enum CellType {
    EMPTY = 0,
    WALL,
    START,
    END,
    VISITED,
    PATH,
};

// 10 x 10 grid
const int ROWS = 10;
const int COLS = 10;

std::vector<std::vector<int>> grid(ROWS, std::vector<int>(COLS, EMPTY));

std::pair<int, int> startNode{0, 0}; 
std::pair<int, int> endNode{9, 9};

void resetGrid(){
    for(int r = 0; r < ROWS; r++){
        for(int c = 0; c < COLS; c++){
            if(grid[r][c] == VISITED || grid[r][c] == PATH)
                grid[r][c] = EMPTY;
        }
    }
    grid[startNode.first][startNode.second] = START;
    grid[endNode.first][endNode.second] = END;
}

void printGrid(const std::vector<std::vector<int>>& grid){
    for (int r = 0; r < ROWS; r++){
        for (int c = 0; c < COLS; c++){
            if (grid[r][c] == START) std::cout << "S ";
            else if (grid[r][c] == END) std::cout << "E ";
            else if (grid[r][c] == WALL) std::cout << "# ";
            else if (grid[r][c] == PATH) std::cout << "* ";
            else if (grid[r][c] == VISITED) std::cout << "o ";
            else std::cout << ". ";
        }
        std::cout << "\n";  // <-- newline after finishing one row
    }
    std::cout << "\n";
}


// is this a valid position to move to
bool isValid(const int r, const int c){
    return (r >= 0 && r < ROWS && c >= 0 && c < COLS && grid[r][c] != WALL);
}

// Breadth first search algo as benchmark
bool bfs(const std::pair<int, int> start, const std::pair<int, int> end){
    std::queue<std::pair<int, int>> q;
    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<std::vector<std::pair<int, int>>> parent(ROWS, std::vector<std::pair<int, int>>(COLS, {-1, -1}));
    int visited_nodes = 0;

    q.push(start);
    visited[start.first][start.second] = true;

    //directions of movement e.g. dr[0] = -1 is {-1, 0, 0 ,0} which means move one row up
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    while(!q.empty()){
        auto [r, c] = q.front(); q.pop();

        visited_nodes++;

        if(std::make_pair(r, c) == end){

            std::pair<int, int> current = end;
            while(current != start){
                grid[current.first][current.second] = PATH;
                current = parent[current.first][current.second];
            }
            std::cout << "Visited cells: " << visited_nodes << std::endl;
            return true;

        }
        for(int i = 0; i < 4; i++){
            // next rows and columns
            int nr = r + dr[i];
            int nc = c + dc[i];
            if(isValid(nr, nc) && !visited[nr][nc]){
                visited[nr][nc] = true;
                parent[nr][nc] = {r, c};
                q.push({nr, nc});
                grid[nr][nc] = VISITED;
            } 
        }
    }
    std::cout << "visited nodes: " << visited_nodes << std::endl;
    return false;
}

// Dijkstra's algorithm next
// need to add weights to grid and use a min-pritority queue 

int getCost(const int r, const int c){
    if(grid[r][c] == WALL){
        return 1e9;           // walls are effectively infinite cost 
    }
    return 1;                 // for current simplicity, cost is uniform
}

bool dijkstra(const std::pair<int, int> start, const std::pair<int, int> end){
    using Node = std::pair<int, std::pair<int, int>>;    //grouping weight, (x-coord, y-coord)

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::vector<int>> distance(ROWS, std::vector<int>(COLS, std::numeric_limits<int>::max()));
    std::vector<std::vector<std::pair<int, int>>> parent(ROWS, std::vector<std::pair<int, int>>(COLS, {-1, -1}));
    int visited_nodes = 0;

    pq.push({0, start});
    distance[start.first][start.second] = 0;

    int dr[] = {-1, 1, 0 , 0};
    int dc[] = {0, 0, -1, 1};

    while(!pq.empty()){
        auto [d, pos] = pq.top(); pq.pop();
        int r = pos.first;
        int c = pos.second;

        visited_nodes++;

        //backtrack by following parent cells to form PATH
        if(std::make_pair(r,c) == end){
            std::pair<int,int> current = end;
            while(current != start){
                grid[current.first][current.second] = PATH;
                current = parent[current.first][current.second];
            }
            std::cout << "Visited nodes: " << visited_nodes << std::endl;
            return true;
        }
        
        if(d > distance[r][c]){
            continue;           // skip if a better path has been found
        }

        for(int i = 0; i < 4; i++){
            int nr = r + dr[i];
            int nc = c + dc[i];
            if(isValid(nr, nc)){
                int newDist = d + getCost(nr, nc);
                if(newDist < distance[nr][nc]){
                    distance[nr][nc] = newDist;
                    parent[nr][nc] = {r, c};
                    pq.push({newDist, {nr, nc}});
                    grid[nr][nc] = VISITED;
                }
            }
        }
    }
    std::cout << "Visited nodes: " << visited_nodes << std::endl;
    return false;

}



// A* algorithm - an optimisation of dijkstra's
// needs a heuristic function to accompany 

int heuristic(const std::pair<int, int>& a, const std::pair<int,int>& b){
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

bool astar(const std::pair<int,int> start, const std::pair<int,int> end){
    using Node = std::pair<int, std::pair<int,int>>;   // group cost and coordinates

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::vector<int>> gScore(ROWS, std::vector<int>(COLS, std::numeric_limits<int>::max()));
    std::vector<std::vector<std::pair<int,int>>> parent(ROWS, std::vector<std::pair<int,int>>(COLS, {-1,-1}));
    int visited_nodes = 0;

    gScore[start.first][start.second] = 0;
    pq.push({heuristic(start,end), start});

    int dr[] = {-1, 1, 0 ,0};
    int dc[] = {0, 0, -1, 1};

    while(!pq.empty()){
        auto [f, pos] = pq.top(); pq.pop();
        int r = pos.first, c = pos.second;

        visited_nodes++;

        if(std::make_pair(r, c) == end){
            std::pair<int,int> current = end;
            while(current != start){
                grid[current.first][current.second] = PATH;
                current = parent[current.first][current.second];
            }
            std::cout << "Visited nodes: " << visited_nodes << std::endl;
            return true;
        }
        
        for(int i = 0; i < 4; i++){
            int nr = r + dr[i];
            int nc = c + dc[i];
            
            if(isValid(nr, nc)){
                int tentative_g = gScore[r][c] + getCost(nr,nc);
                if(tentative_g < gScore[nr][nc]){
                    gScore[nr][nc] = tentative_g;
                    int fScore = tentative_g + heuristic({nr, nc}, end);
                    parent[nr][nc] = {r,c};
                    pq.push({fScore, {nr,nc}});
                    grid[nr][nc] = VISITED;
                }
            }
        
        }
    
    }
    std::cout << "Visited nodes: " << visited_nodes << std::endl;
    return false;


}



int main() {

    grid[startNode.first][startNode.second] = START;
    grid[endNode.first][endNode.second] = END;

    grid[0][2] = WALL; grid[0][3] = WALL; grid[0][7] = WALL;
    grid[1][2] = WALL; grid[1][5] = WALL; grid[1][7] = WALL;
    grid[2][1] = WALL; grid[2][2] = WALL; grid[2][4] = WALL; grid[2][5] = WALL;
    grid[3][1] = WALL; grid[3][6] = WALL; grid[3][7] = WALL;
    grid[4][1] = WALL; grid[4][3] = WALL; grid[4][4] = WALL; grid[4][8] = WALL;
    grid[5][3] = WALL; grid[5][6] = WALL;
    grid[6][0] = WALL; grid[6][1] = WALL; grid[6][3] = WALL; grid[6][5] = WALL; grid[6][6] = WALL; grid[6][8] = WALL;
    // no walls on row 7
    grid[8][1] = WALL; grid[8][2] = WALL; grid[8][3] = WALL; grid[8][4] = WALL;
    grid[8][5] = WALL; grid[8][6] = WALL; grid[8][7] = WALL; grid[8][8] = WALL;


    std::cout << "Initial Grid \n";
    printGrid(grid);

    auto startTime = std::chrono::high_resolution_clock::now();
    bool pathFound = astar(startNode, endNode);
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

    if(pathFound){
        std::cout << "Path found! Time Elapsed: " << duration << "us" <<std::endl;
    }
    else{
        std::cout << "Path not found :(" << std::endl;
    }


    grid[startNode.first][startNode.second] = START;
    grid[endNode.first][endNode.second] = END;

    printGrid(grid);
    resetGrid();

    return 0;

}