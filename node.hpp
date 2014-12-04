#ifndef NODE_HPP
#define NODE_HPP
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

using namespace std;

const int MAP_HORIZONTAL =60; // horizontal size of the main_map
const int MAP_VERTICAL=60; // vertical size size of the main_map
static int main_map[MAP_HORIZONTAL][MAP_VERTICAL];
static int closed_nodes_map[MAP_HORIZONTAL][MAP_VERTICAL]; // main_map of closed (tried-out) nodes
static int open_nodes_map[MAP_HORIZONTAL][MAP_VERTICAL]; // main_map of open (not-yet-tried) nodes
static int dir_map[MAP_HORIZONTAL][MAP_VERTICAL]; // main_map of directions
const int DIRECTIONS_TO_MOVE=8; // number of possible directions to go at any position
// if DIRECTIONS_TO_MOVE==4
//static int dx[DIRECTIONS_TO_MOVE]={1, 0, -1, 0};
//static int dy[DIRECTIONS_TO_MOVE]={0, 1, 0, -1};
// if DIRECTIONS_TO_MOVE==8
static int dx[DIRECTIONS_TO_MOVE]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[DIRECTIONS_TO_MOVE]={0, 1, 1, 1, 0, -1, -1, -1};

class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        node(int xp, int yp, int d, int p) 
            {xPos=xp; yPos=yp; level=d; priority=p;}
    
        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority=level+estimate(xDest, yDest)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i) // i: direction
        {
             level+=(DIRECTIONS_TO_MOVE==8?(i%2==0?10:14):10);
        }
        
        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;         

            // Euclidian Distance
            d=static_cast<int>(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            //d=abs(xd)+abs(yd);
            
            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart, 
                 const int & xFinish, const int & yFinish )
{
    static priority_queue<node> NOT_TRIED_NODES[2]; // list of open (not-yet-tried) nodes
    static int NT_INDEX; // NOT_TRIED_NODES index
    static node* n0;
    static node* m0;
    static int i, j, x, y, X_DIR_X, ydy;
    static char c;
    NT_INDEX=0;

    // reset the node maps
    for(y=0;y<MAP_VERTICAL;y++)
    {
        for(x=0;x<MAP_HORIZONTAL;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    NOT_TRIED_NODES[NT_INDEX].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes main_map

    // A* search
    while(!NOT_TRIED_NODES[NT_INDEX].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( NOT_TRIED_NODES[NT_INDEX].top().getxPos(), NOT_TRIED_NODES[NT_INDEX].top().getyPos(), 
                     NOT_TRIED_NODES[NT_INDEX].top().getLevel(), NOT_TRIED_NODES[NT_INDEX].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        NOT_TRIED_NODES[NT_INDEX].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes main_map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+DIRECTIONS_TO_MOVE/2)%DIRECTIONS_TO_MOVE;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!NOT_TRIED_NODES[NT_INDEX].empty()) NOT_TRIED_NODES[NT_INDEX].pop();           
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<DIRECTIONS_TO_MOVE;i++)
        {
            X_DIR_X=x+dx[i]; ydy=y+dy[i];

            if(!(X_DIR_X<0 || X_DIR_X>MAP_HORIZONTAL-1 || ydy<0 || ydy>MAP_VERTICAL-1 || main_map[X_DIR_X][ydy]==1 
                || closed_nodes_map[X_DIR_X][ydy]==1))
            {
                // generate a child node
                m0=new node( X_DIR_X, ydy, n0->getLevel(), 
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[X_DIR_X][ydy]==0)
                {
                    open_nodes_map[X_DIR_X][ydy]=m0->getPriority();
                    NOT_TRIED_NODES[NT_INDEX].push(*m0);
                    // mark its parent node direction
                    dir_map[X_DIR_X][ydy]=(i+DIRECTIONS_TO_MOVE/2)%DIRECTIONS_TO_MOVE;
                }
                else if(open_nodes_map[X_DIR_X][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[X_DIR_X][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[X_DIR_X][ydy]=(i+DIRECTIONS_TO_MOVE/2)%DIRECTIONS_TO_MOVE;

                    // replace the node
                    // by emptying one NOT_TRIED_NODES to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(NOT_TRIED_NODES[NT_INDEX].top().getxPos()==X_DIR_X && 
                           NOT_TRIED_NODES[NT_INDEX].top().getyPos()==ydy))
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
                    NOT_TRIED_NODES[NT_INDEX].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

int main()
{
    srand(time(NULL));

    // create empty main_map
    for(int y=0;y<MAP_VERTICAL;y++)
    {
        for(int x=0;x<MAP_HORIZONTAL;x++) main_map[x][y]=0;
    }

    // fillout the main_map matrix with a '+' pattern
    for(int x=MAP_HORIZONTAL/8;x<MAP_HORIZONTAL*7/8;x++)
    {
        main_map[x][MAP_VERTICAL/2]=1;
    }
    for(int y=MAP_VERTICAL/8;y<MAP_VERTICAL*7/8;y++)
    {
        main_map[MAP_HORIZONTAL/2][y]=1;
    }
    
    // randomly select start and finish locations
    int xA, yA, xB, yB;
    switch(rand()%8)
    {
        case 0: xA=0;yA=0;xB=MAP_HORIZONTAL-1;yB=MAP_VERTICAL-1; break;
        case 1: xA=0;yA=MAP_VERTICAL-1;xB=MAP_HORIZONTAL-1;yB=0; break;
        case 2: xA=MAP_HORIZONTAL/2-1;yA=MAP_VERTICAL/2-1;xB=MAP_HORIZONTAL/2+1;yB=MAP_VERTICAL/2+1; break;
        case 3: xA=MAP_HORIZONTAL/2-1;yA=MAP_VERTICAL/2+1;xB=MAP_HORIZONTAL/2+1;yB=MAP_VERTICAL/2-1; break;
        case 4: xA=MAP_HORIZONTAL/2-1;yA=0;xB=MAP_HORIZONTAL/2+1;yB=MAP_VERTICAL-1; break;
        case 5: xA=MAP_HORIZONTAL/2+1;yA=MAP_VERTICAL-1;xB=MAP_HORIZONTAL/2-1;yB=0; break;
        case 6: xA=0;yA=MAP_VERTICAL/2-1;xB=MAP_HORIZONTAL-1;yB=MAP_VERTICAL/2+1; break;
        case 7: xA=MAP_HORIZONTAL-1;yA=MAP_VERTICAL/2+1;xB=0;yB=MAP_VERTICAL/2-1; break;
    }

    cout<<"Map Size (X,Y): "<<MAP_HORIZONTAL<<","<<MAP_VERTICAL<<endl;
    cout<<"Start: "<<xA<<","<<yA<<endl;
    cout<<"Finish: "<<xB<<","<<yB<<endl;
    // get the route
    clock_t start = clock();
    string route=pathFind(xA, yA, xB, yB);
    if(route=="") cout<<"An empty route generated!"<<endl;
    clock_t end = clock();
    double time_elapsed = double(end - start);
    cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
    cout<<"Route:"<<endl;
    cout<<route<<endl<<endl;

    // follow the route on the main_map and display it 
    if(route.length()>0)
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
        for(int y=0;y<MAP_VERTICAL;y++)
        {
            for(int x=0;x<MAP_HORIZONTAL;x++)
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
        }
    }
    getchar(); // wait for a (Enter) keypress  
    return(0);
}

#endif // NODE_HPP
