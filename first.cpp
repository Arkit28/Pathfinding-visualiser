#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <limits>
#include <chrono>
#include <thread> 
#include <cstdlib>
#include <ctime>
#include <stack>
#include <algorithm>
#include <random>

namespace ansi {
    constexpr const char* CLEAR = "\x1b[2J";
    constexpr const char* HOME = "\x1b[H";
    constexpr const char* HIDE = "\x1b[?25l";
    constexpr const char* SHOW = "\x1b[25h";
    constexpr const char* RESET = "\x1b[0m";
    constexpr const char* WALLC = "\x1b[32m";
    constexpr const char* STARTC = "\x1b[31m";
    constexpr const char* ENDC = "\x1b[31m";
    constexpr const char* VISITC = "\x1b[34m";
    constexpr const char* PATHC = "\x1b[33m";    
}

inline void clearConsole() {
    std::cout << ansi::CLEAR << ansi::HOME;
}

inline void showCursor(bool on){
    std::cout << (on ? ansi::SHOW : ansi::HIDE);
}

#ifdef _WIN32
#include <windows.h>
#endif

void enableANSI(){
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if(hOut != INVALID_HANDLE_VALUE){
        DWORD dwMode = 0;
        if(GetConsoleMode(hOut, &dwMode)){
            SetConsoleMode(hOut, dwMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
        }
    }
#endif
}


enum CellType {
    EMPTY = 0,
    WALL,
    START,
    END,
    VISITED,
    PATH,
};

// 10 x 10 grid
const int ROWS = 25;
const int COLS = 25;

std::vector<std::vector<int>> grid(ROWS, std::vector<int>(COLS, EMPTY));

std::pair<int, int> startNode{0, 0}; 
std::pair<int, int> endNode{24, 24};

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
        std::cout << "\n";  
    }
    std::cout << "\n";
}

void printGridColoured(const std::vector<std::vector<int>>& grid){
    for(int r = 0; r < ROWS; r++){
        for(int c = 0; c < COLS; c++){
            switch(grid[r][c]){
                case START: std::cout << ansi::STARTC << "S " << ansi::RESET; break;
                case END:   std::cout << ansi::ENDC << "E " << ansi::RESET; break;
                case WALL:  std::cout << ansi::WALLC << "# " << ansi::RESET; break;
                case PATH:  std::cout << ansi::PATHC << "* " << ansi::RESET; break;
                case VISITED: std::cout << ansi::VISITC << "o " << ansi::RESET; break;
                default: std::cout << ". "; 
            }
        }
        std::cout << "\n";
    }
}

void renderFrame(const char* title, int visitedCount, int delay_us){
    std::cout << ansi::HOME;
    std::cout << title << "\n\n";
    printGridColoured(grid);
    std::cout << "\nVisited: " << visitedCount << "\n";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}


// is this a valid position to move to
bool isValid(const int r, const int c){
    return (r >= 0 && r < ROWS && c >= 0 && c < COLS && grid[r][c] != WALL);
}

// Breadth first search algo as benchmark
int bfs(const std::pair<int, int>& start, const std::pair<int, int>& end, bool animate=true, int delay_us=50){
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

        if(animate){
            renderFrame("BFS", visited_nodes, delay_us);
        }

        if(std::make_pair(r, c) == end){

            std::pair<int, int> current = end;
            while(current != start){
                grid[current.first][current.second] = PATH;
                current = parent[current.first][current.second];
                if(animate){
                    renderFrame("BFS - path construction through backtracking", visited_nodes, delay_us);
                }
            }
            return visited_nodes;

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
                if(animate){
                    renderFrame("BFS", visited_nodes, delay_us);
                }
            } 
        }
    }
    return visited_nodes;
}


int dfs(const std::pair<int, int>& start, const std::pair<int, int>& end, bool animate=true, int delay_us=50){
    using Node = std::pair<int, std::pair<int,int>>;
    std::stack<std::pair<int,int>> st;
    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<std::vector<std::pair<int, int>>> parent(ROWS, std::vector<std::pair<int, int>>(COLS, {-1, -1}));

    st.push( startNode);
    visited[start.first][start.second] = true;
    parent[start.first][start.second] = {-1,-1};
    int visited_nodes = 0;

    int dr[] = {-1, 1, 0 , 0};
    int dc[] = {0, 0, -1, 1};

    while(!st.empty()){
        auto [x, y] = st.top(); st.pop();

        visited_nodes++;

        if(std::make_pair(x, y) == end){
            std::pair<int,int> current = end;
            while(current != start){
                grid[current.first][current.second] = PATH;
                current = parent[current.first][current.second];
                if(animate){
                    renderFrame("DFS Path reconstruction through backtracking", visited_nodes, delay_us);
                }
            }
            return visited_nodes;

        }

        for(int i = 0; i < 4; i++){
            int nx = x + dr[i], ny = y + dc[i];
            if(isValid(nx, ny) && !visited[nx][ny]){
                visited[nx][ny] = true;
                st.push({nx,ny});
                parent[nx][ny] = {x,y};
                grid[nx][ny] = VISITED;
                if(animate){
                    renderFrame("DFS", visited_nodes, delay_us);
                }

            }
        }
    }
    return visited_nodes;
}

// Dijkstra's algorithm next
// need to add weights to grid and use a min-pritority queue 

int getCost(const int r, const int c){
    if(grid[r][c] == WALL){
        return 1e9;           // walls are effectively infinite cost 
    }
    return 1;                 // for current simplicity, cost is uniform
}

bool dijkstra(const std::pair<int, int> start, const std::pair<int, int> end, bool animate=true, int delay_us=50){
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
        if(animate){
            renderFrame("Dijkstra", visited_nodes, delay_us);
        }

        //backtrack by following parent cells to form PATH
        if(std::make_pair(r,c) == end){
            std::pair<int,int> current = end;
            while(current != start){
                grid[current.first][current.second] = PATH;
                current = parent[current.first][current.second];
                if(animate){
                renderFrame("Dijkstra - Path reconstruction through backtracking", visited_nodes, delay_us);
                }
                
            }
            return visited_nodes;
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
                    if(animate){
                        renderFrame("Dijkstra ", visited_nodes, delay_us);
                    }
                }
            }
        }
    }
    return visited_nodes;


}



// A* algorithm - an optimisation of dijkstra's
// needs a heuristic function to accompany 

int heuristic(const std::pair<int, int>& a, const std::pair<int,int>& b){
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

bool astar(const std::pair<int,int>& start, const std::pair<int,int>& end, bool animate=true, int delay_us=50){
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
        if(animate){
            renderFrame("A* ", visited_nodes, delay_us);
        }

        if(std::make_pair(r, c) == end){
            std::pair<int,int> current = end;
            while(current != start){
                grid[current.first][current.second] = PATH;
                current = parent[current.first][current.second];
                if(animate){
                    renderFrame("A* - Path reconstruction through backtracking", visited_nodes, delay_us);
                }
            }
            return visited_nodes;

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
                    if(animate){
                        renderFrame("A* ", visited_nodes, delay_us);
                    }
                }
            }
        
        }
    
    }   
    return visited_nodes;

}

bool inBounds(int r, int c){
    return r >= 0 && r < ROWS && c >= 0 && c < COLS;
}

void MazeGeneration() {
    int dr [] = {-2, 2, 0 , 0};
    int dc [] = {0, 0, -2, 2};

    // fill grid with walls
    for(int i = 0; i < ROWS; i++){
        for(int j = 0; j < COLS; j++){
            grid[i][j] = WALL;
        }
    }

    std::stack<std::pair<int, int>> st;
    st.push(startNode);
    grid[startNode.first][startNode.second] = EMPTY;

    bool reachedEnd = false;

    static std::default_random_engine rng(static_cast<unsigned>(std::time(nullptr)));

    while(!st.empty()){
        auto [r, c] = st.top();
        if(std::make_pair(r, c) == endNode) {
            reachedEnd = true; // ✅ mark end reached
        }

        std::vector<int> dirs = {0, 1, 2, 3};
        std::shuffle(dirs.begin(), dirs.end(), rng);

        bool moved = false;
        for(int d : dirs){
            int nr = r + dr[d];
            int nc = c + dc[d];
            if(inBounds(nr, nc) && grid[nr][nc] == WALL){
                grid[r + dr[d]/2][c + dc[d]/2] = EMPTY;
                grid[nr][nc] = EMPTY;
                st.push({nr, nc});
                moved = true;
                break;
            }
        }
        if(!moved){
            st.pop();
        }
    }

    // Make sure start/end are visible
    grid[startNode.first][startNode.second] = START;
    grid[endNode.first][endNode.second] = END;

    // ✅ If the end wasn’t reached during carving, force-connect it
    if(!reachedEnd){
        // Simple way: carve a direct corridor between start and end
        int r = startNode.first, c = startNode.second;
        while(r != endNode.first){
            r += (endNode.first > r ? 1 : -1);
            grid[r][c] = EMPTY;
        }
        while(c != endNode.second){
            c += (endNode.second > c ? 1 : -1);
            grid[r][c] = EMPTY;
        }
        grid[startNode.first][startNode.second] = START;
        grid[endNode.first][endNode.second] = END;
    }
}



template<typename Function>

void AlgorithmEvaluation(const std::string& name, Function algorithm){
    auto gridCopy = grid;
    resetGrid();

    std::cout << "\n " << name << " Algorithm \n";
    std::cout << "Press Enter to start...";
    std::cin.get();

    clearConsole();

    auto startTime = std::chrono::high_resolution_clock::now();
    
    int visitedCount = algorithm(startNode, endNode, true, 100); // 100ms delay for better visibility
    

    auto endTime = std::chrono::high_resolution_clock::now();
    
    // Use microseconds for more precise timing
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

    std::cout << "\n" << name << " Results:\n";
    printGridColoured(grid);
    std::cout << "Visited Nodes: " << visitedCount << "\n";
    
    if(duration >= 1000){
        std::cout << "Time Taken: " << duration / 1000.0 << " ms\n";
    } 
    else{
        std::cout << "Time Taken: " << duration << " μs\n";
    }
    
    std::cout << "\nPress Enter to continue...";
    std::cin.get();
}



int main() {

    enableANSI();
    std::cout << std::string(3, '\n');

    grid[startNode.first][startNode.second] = START;
    grid[endNode.first][endNode.second] = END;

    MazeGeneration();
    
    showCursor(false);

    std::cout << "Initial Grid\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printGridColoured(grid);

    
    AlgorithmEvaluation("BFS", bfs);
    AlgorithmEvaluation("DFS", dfs);
    AlgorithmEvaluation("Dijkstra", dijkstra);
    AlgorithmEvaluation("A*", astar);


    showCursor(true);
    
}


/*
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

    //wall placements:
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
*/ 