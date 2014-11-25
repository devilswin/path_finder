// Astar.cpp
// http://en.wikipedia.org/wiki/A*
// Compiler: Dev-C++ 4.9.9.2
// FB - 201012256
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
using namespace std;

class A_STAR_PATH
{
private:
    
    int map_x, map_y;
    vector<vector<int>> main_map;
    vector<vector<int>> closed_nodes_map;  // main_map of closed (tried-out) nodes
    vector<vector<int>> open_nodes_map; // main_map of open (not-yet-tried) nodes
    vector<vector<int>> dir_map; // main_map of directions
    int DIRECTIONS_TO_MOVE; // number of possible directions to go at any position
    
    vector<int> dx = {1, 1, 0, -1, -1, -1, 0, 1};
    vector<int> dy = {0, 1, 1, 1, 0, -1, -1, -1};
public:
    A_STAR_PATH(int y, int x, vector<vector<int>> map, const int possible_moves);
    string pathFind(int  xStart, int yStart, 
                    int xFinish,  int yFinish);
    
};

class node
{
    // current position
    int X_CORD;
    int Y_CORD;
    // total distance already travelled to reach the node
    int DIS_TRAVEL;
    // PRIORITY=DIS_TRAVEL+remaining distance estimate
    int PRIORITY;  // smaller: higher PRIORITY
    
public:
    node(int xp, int yp, int d, int p) 
    {X_CORD=xp; Y_CORD=yp; DIS_TRAVEL=d; PRIORITY=p;}
    
    int getX_CORD() const {return X_CORD;}
    int getY_CORD() const {return Y_CORD;}
    int getDIS_TRAVEL() const {return DIS_TRAVEL;}
    int getPRIORITY() const {return PRIORITY;}
    
    void updatePRIORITY( int xDest, int yDest)
    {
        PRIORITY=DIS_TRAVEL+estimate(xDest, yDest, 2)*10; //A*
    }
    
    // give better PRIORITY to going strait instead of diagonally
    void nextDIS_TRAVEL(int i, int POSSIBLE_MOVES) // i: direction
    {
        DIS_TRAVEL+=(POSSIBLE_MOVES==8?(i%2==0?10:14):10);
    }
    
    // Estimation function for the remaining distance to the goal.
    int estimate(int xDest, int yDest, const int type)
    {
        static int xd, yd, d;
        xd=xDest-X_CORD;
        yd=yDest-Y_CORD;         
        
        if(type==1)
            // Euclidian Distance
            d=static_cast<int>(sqrt(xd*xd+yd*yd));
        else if(type == 2)
            // Manhattan distance
            d=abs(xd)+abs(yd);
        
        else if (type == 3)
            //Chebyshev distance
            d=max(abs(xd), abs(yd));
        else
            exit(type);
        return(d);
    }
};

// Determine PRIORITY (in the PRIORITY queue)
bool operator<(node a, node b)
{
    return a.getPRIORITY() > b.getPRIORITY();
}

// A-star algorithm.
// The route returned is a string of direction digits.
A_STAR_PATH::A_STAR_PATH(int y, int x, vector<vector<int> > map, const int possible_moves)
{
    map_x = x;
    map_y = y;
    
    for(int q = 0; q < y; q++)
    {
        vector<int> row;
        vector<int> row2;
        for(int z = 0; z< x; z++)
        {
            row.push_back(map[q][z]);
            row2.push_back(0);
        }
        main_map.push_back(row);
        dir_map.push_back(row2);
    }
    DIRECTIONS_TO_MOVE = possible_moves;
    
    
    
}

string A_STAR_PATH::pathFind(int  xStart, int yStart, 
                             int xFinish,  int yFinish)
{
    static priority_queue<node> NOT_TRIED_NODES[2]; // list of open (not-yet-tried) nodes
    static int NT_INDEX; // NOT_TRIED_NODES index
    static node* NODE_ONE;
    static node* NODE_TWO;
    static int i, j, xdx, ydy;
    static int x, y; 
    static char c;
    NT_INDEX = 0;
    
    // reset the node maps
    for(y=0;y< map_y;y++)
    {
        vector <int> row1, row2;
        
        for(x=0;x< map_x;x++)
        {
            row1.push_back(0);
            row2.push_back(0);
        }
        open_nodes_map.push_back(row1);
        closed_nodes_map.push_back(row2);
    }
    
    // create the start node and push into list of open nodes
    
    NODE_ONE =new node(xStart, yStart, 0, 0);
    NODE_ONE->updatePRIORITY(xFinish, yFinish);
    NOT_TRIED_NODES[NT_INDEX].push(*NODE_ONE);
    
    open_nodes_map[yStart][xStart]=NODE_ONE->getPRIORITY(); // mark it on the open nodes main_map
    
    // A* search
    while(!NOT_TRIED_NODES[NT_INDEX].empty())
    {
        // get the current node w/ the highest PRIORITY
        // from the list of open nodes
        NODE_ONE=new node( NOT_TRIED_NODES[NT_INDEX].top().getX_CORD(), NOT_TRIED_NODES[NT_INDEX].top().getY_CORD(), 
                           NOT_TRIED_NODES[NT_INDEX].top().getDIS_TRAVEL(), NOT_TRIED_NODES[NT_INDEX].top().getPRIORITY());
        
        x=NODE_ONE->getX_CORD(); y=NODE_ONE->getY_CORD();
        
        NOT_TRIED_NODES[NT_INDEX].pop(); // remove the node from the open list
        open_nodes_map[y][x]=0;
        
        // mark it on the closed nodes main_map
        closed_nodes_map[y][x]=1;
        
        // quit searching when the goal state is reached
        //if((*NODE_ONE).ete(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[y][x];
                c='0'+(j+DIRECTIONS_TO_MOVE/2)%DIRECTIONS_TO_MOVE;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }
            
            // garbage collection
            delete NODE_ONE;
            // empty the leftover nodes
            while(!NOT_TRIED_NODES[NT_INDEX].empty()) NOT_TRIED_NODES[NT_INDEX].pop();           
            return path;
        }
        
        // generate moves (child nodes) in all possible directions
        for(i=0;i<DIRECTIONS_TO_MOVE;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];
            
            if(!(xdx<0 || xdx>map_x-1 || ydy<0 || ydy>map_y-1 || main_map[ydy][xdx]==1 
                 || closed_nodes_map[ydy][xdx]==1))
            {
                // generate a child node
                NODE_TWO=new node( xdx, ydy, NODE_ONE->getDIS_TRAVEL(), 
                                   NODE_ONE->getPRIORITY());
                NODE_TWO->nextDIS_TRAVEL(i, DIRECTIONS_TO_MOVE);
                NODE_TWO->updatePRIORITY(xFinish, yFinish);
                
                // if it is not in the open list then add into that
                if(open_nodes_map[ydy][xdx]==0)
                {
                    open_nodes_map[ydy][xdx]=NODE_TWO->getPRIORITY();
                    NOT_TRIED_NODES[NT_INDEX].push(*NODE_TWO);
                    // mark its parent node direction
                    dir_map[ydy][xdx]=(i+DIRECTIONS_TO_MOVE/2)%DIRECTIONS_TO_MOVE;
                }
                else if(open_nodes_map[ydy-1][xdx-1]>NODE_TWO->getPRIORITY())
                {
                    // update the PRIORITY info
                    open_nodes_map[ydy][xdx]=NODE_TWO->getPRIORITY();
                    // update the parent direction info
                    dir_map[ydy][xdx]=(i+DIRECTIONS_TO_MOVE/2)%DIRECTIONS_TO_MOVE;
                    
                    // replace the node
                    // by emptying one NOT_TRIED_NODES to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(NOT_TRIED_NODES[NT_INDEX].top().getX_CORD()==xdx && 
                            NOT_TRIED_NODES[NT_INDEX].top().getY_CORD()==ydy))
                    {                
                        NOT_TRIED_NODES[1-NT_INDEX].push(NOT_TRIED_NODES[NT_INDEX].top());
                        NOT_TRIED_NODES[NT_INDEX].pop();       
                    }
                    NOT_TRIED_NODES[NT_INDEX].pop(); // remove the wanted node
                    
                    // empty the larger size NOT_TRIED_NODES to the smaller one
                    if(NOT_TRIED_NODES[NT_INDEX].size()>NOT_TRIED_NODES[1-NT_INDEX].size()) NT_INDEX=1-NT_INDEX;
                    while(!NOT_TRIED_NODES[NT_INDEX].empty())
                    {                
                        NOT_TRIED_NODES[1-NT_INDEX].push(NOT_TRIED_NODES[NT_INDEX].top());
                        NOT_TRIED_NODES[NT_INDEX].pop();       
                    }
                    NT_INDEX=1-NT_INDEX;
                    NOT_TRIED_NODES[NT_INDEX].push(*NODE_TWO); // add the better node instead
                }
                else {delete NODE_TWO;NODE_TWO = nullptr;} // garbage collection
            }
        }
        delete NODE_ONE;
  
        NODE_ONE = nullptr;
        // garbage collection
    }
    return ""; // no route found
}

int main()
{
    srand(time(NULL));
    sf::Vector2i MAP_DIMENSIONS(60,60);
    vector<vector<int>> start_map(MAP_DIMENSIONS.x, vector<int>(MAP_DIMENSIONS.y));
    // create empty main_map
    for(int y=0;y<MAP_DIMENSIONS.y;y++)
    {
        for(int x=0;x<MAP_DIMENSIONS.x;x++) start_map[x][y]=0;
    }
    
    // fillout the main_map matrix with a '+' pattern
    for(int x=MAP_DIMENSIONS.x/8;x<MAP_DIMENSIONS.x*7/8;x++)
    {
        start_map[x][MAP_DIMENSIONS.y/2]=1;
    }
    for(int y=MAP_DIMENSIONS.y/8;y<MAP_DIMENSIONS.y*7/8;y++)
    {
        start_map[MAP_DIMENSIONS.x/2][y]=1;
    }
    
    // randomly select start and finish locations
    int xA, yA, xB, yB;
    switch(rand()%8)
    {
    case 0: xA=0;yA=0;xB=MAP_DIMENSIONS.x-1;yB=MAP_DIMENSIONS.y-1; break;
    case 1: xA=0;yA=MAP_DIMENSIONS.y-1;xB=MAP_DIMENSIONS.x-1;yB=0; break;
    case 2: xA=MAP_DIMENSIONS.x/2-1;yA=MAP_DIMENSIONS.y/2-1;xB=MAP_DIMENSIONS.x/2+1;yB=MAP_DIMENSIONS.y/2+1; break;
    case 3: xA=MAP_DIMENSIONS.x/2-1;yA=MAP_DIMENSIONS.y/2+1;xB=MAP_DIMENSIONS.x/2+1;yB=MAP_DIMENSIONS.y/2-1; break;
    case 4: xA=MAP_DIMENSIONS.x/2-1;yA=0;xB=MAP_DIMENSIONS.x/2+1;yB=MAP_DIMENSIONS.y-1; break;
    case 5: xA=MAP_DIMENSIONS.x/2+1;yA=MAP_DIMENSIONS.y-1;xB=MAP_DIMENSIONS.x/2-1;yB=0; break;
    case 6: xA=0;yA=MAP_DIMENSIONS.y/2-1;xB=MAP_DIMENSIONS.x-1;yB=MAP_DIMENSIONS.y/2+1; break;
    case 7: xA=MAP_DIMENSIONS.x-1;yA=MAP_DIMENSIONS.y/2+1;xB=0;yB=MAP_DIMENSIONS.y/2-1; break;
    }
    
    cout<<"Map Size (X,Y): "<<MAP_DIMENSIONS.x<<","<<MAP_DIMENSIONS.y<<endl;
    cout<<"Start: "<<xA<<","<<yA<<endl;
    cout<<"Finish: "<<xB<<","<<yB<<endl;
    // get the route
    clock_t start = clock();
    A_STAR_PATH path(MAP_DIMENSIONS.x,MAP_DIMENSIONS.y,start_map,8);
    string route=path.pathFind(xA, yA, xB, yB);
    
    if(route=="") cout<<"An empty route generated!"<<endl;
    clock_t end = clock();
    double time_elapsed = double(end - start);
    cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
    cout<<"Route:"<<endl;
    cout<<route<<endl<<endl;
    
    
    // follow the route on the main_map and display it 
    /*  if(route.length()>0)
    {
        int j; char c;
        int x=xA;
        int y=yA;
        main_map[x][y]=2;
        for(int i=0;i<route.length();i++)
        {
            c =route.at(i);
            j=atoi(&c); 
            x=x+dx[j];
            y=y+dy[j];
            main_map[x][y]=3;
        }
        main_map[x][y]=4;
        
        // display the main_map with the route
  /     for(int y=0;y<MAP_DIMENSIONS.y;y++)
        {
            for(int x=0;x<MAP_DIMENSIONS.x;x++)
                if(main_map[x][y]==0)
                    cout<<".";
                else if(main_map[x][y]==1)
                    cout<<"O"; //obstacle
                else if(main_map[x][y]==2)
                    cout<<"S"; //start
                else if(main_map[x][y]==3)
                    cout<<"R"; //route
                else if(main_map[x][y]==4)
                    cout<<"F"; //finish
            cout<<endl;
        }*/
    //}
    //getchar(); // wait for a (Enter) keypress  
    return(0);
}