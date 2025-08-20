#include <iostream>
#include <vector>
#include <queue>
#include <utility>

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
bool isValid(int r, int c){
    return (r >= 0 && r < ROWS && c >= 0 && c < COLS && grid[r][c] != WALL);
}

// Breadth first search algo as benchmark
bool bfs(std::pair<int, int> start, std::pair<int, int> end){
    std::queue<std::pair<int, int>> q;
    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<std::vector<std::pair<int, int>>> parent(ROWS, std::vector<std::pair<int, int>>(COLS, {-1, -1}));

    q.push(start);
    visited[start.first][start.second] = true;

    //directions of movement e.g. dr[0] = -1 is {-1, 0, 0 ,0} which means move one row up
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    while(!q.empty()){
        auto [r, c] = q.front(); q.pop();

        if(std::make_pair(r, c) == end){

            std::pair<int, int> current = end;
            while(current != start){
                grid[current.first][current.second] = PATH;
                current = parent[current.first][current.second];
            }
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
    return false;
}

// Dijkstra's algorithm next

int main() {

    grid[startNode.first][startNode.second] = START;
    grid[endNode.first][endNode.second] = END;

    grid[1][0] = WALL;
    grid[1][1] = WALL;
    grid[1][2] = WALL;
    grid[2][2] = WALL;
    grid[3][2] = WALL;
    grid[4][2] = WALL;
    grid[5][2] = WALL;
    grid[6][2] = WALL;
    grid[7][2] = WALL;
    grid[8][2] = WALL;
    grid[9][2] = WALL;


    std::cout << "Initial Grid \n";
    printGrid(grid);

    if(bfs(startNode, endNode))    std::cout << "Path found: \n";
    else                           std::cout << "No path found :(\n";

    grid[startNode.first][startNode.second] = START;
    grid[endNode.first][endNode.second] = END;

    printGrid(grid);

    return 0;

}