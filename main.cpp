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
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <list>
#include "Animation.hpp"
#include "AnimatedSprite.hpp"
#include <chrono>
#include <thread>
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
    string route;
    
public:
    A_STAR_PATH(int y, int x, vector<vector<int>> map, const int possible_moves);
    string pathFind(int  xStart, int yStart, 
                    int xFinish,  int yFinish, int move_tile);
    int x_cords; int y_cords;
    vector<int> dx = {1, 1, 0, -1, -1, -1, 0, 1};
    vector<int> dy = {0, 1, 1, 1, 0, -1, -1, -1};
    
    
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
        PRIORITY=DIS_TRAVEL+estimate(xDest, yDest, 1)*10; //A*
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
                             int xFinish,  int yFinish, int move_tile)
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
            NODE_ONE = nullptr;
            // empty the leftover nodes
            while(!NOT_TRIED_NODES[NT_INDEX].empty()) NOT_TRIED_NODES[NT_INDEX].pop();    
            route = path;
            return path;
        }
        
        // generate moves (child nodes) in all possible directions
        for(i=0;i<DIRECTIONS_TO_MOVE;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];
            
            if(!(xdx<0 || xdx>map_x-1 || ydy<0 || ydy>map_y-1 || main_map[ydy][xdx]!=move_tile 
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
                else if(open_nodes_map[ydy][xdx]>NODE_TWO->getPRIORITY())
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
                else {
                    delete NODE_TWO;
                    NODE_TWO = NULL;
                } // garbage collection
            }
        }
        delete NODE_ONE;
        
        NODE_ONE = NULL;
        // garbage collection
    }
    return ""; // no route found
}



class TileMap : public sf::Drawable, public sf::Transformable
{
public:
    
    bool load(const std::string& tileset, sf::Vector2u tileSize, const int* tiles, unsigned int width, unsigned int height)
    {
        // load the tileset texture
        if (!m_tileset.loadFromFile(tileset))
            return false;
        
        // resize the vertex array to fit the level size
        m_vertices.setPrimitiveType(sf::Quads);
        m_vertices.resize(width * height * 4);
        
        // populate the vertex array, with one quad per tile
        for (unsigned int i = 0; i < width; ++i)
            for (unsigned int j = 0; j < height; ++j)
            {
                // get the current tile number
                int tileNumber = tiles[i + j * width];
                
                // find its position in the tileset texture
                int tu = tileNumber % (m_tileset.getSize().x / tileSize.x);
                int tv = tileNumber / (m_tileset.getSize().x / tileSize.x);
                
                // get a pointer to the current tile's quad
                sf::Vertex* quad = &m_vertices[(i + j * width) * 4];
                
                // define its 4 corners
                quad[0].position = sf::Vector2f(i * tileSize.x, j * tileSize.y);
                quad[1].position = sf::Vector2f((i + 1) * tileSize.x, j * tileSize.y);
                quad[2].position = sf::Vector2f((i + 1) * tileSize.x, (j + 1) * tileSize.y);
                quad[3].position = sf::Vector2f(i * tileSize.x, (j + 1) * tileSize.y);
                
                // define its 4 texture coordinates
                quad[0].texCoords = sf::Vector2f(tu * tileSize.x, tv * tileSize.y);
                quad[1].texCoords = sf::Vector2f((tu + 1) * tileSize.x, tv * tileSize.y);
                quad[2].texCoords = sf::Vector2f((tu + 1) * tileSize.x, (tv + 1) * tileSize.y);
                quad[3].texCoords = sf::Vector2f(tu * tileSize.x, (tv + 1) * tileSize.y);
            }
        
        return true;
    }
    
private:
    
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        // apply the transform
        states.transform *= getTransform();
        
        // apply the tileset texture
        states.texture = &m_tileset;
        
        // draw the vertex array
        target.draw(m_vertices, states);
    }
    
    sf::VertexArray m_vertices;
    sf::Texture m_tileset;
};

int main()
{
    // create the window
    sf::Vector2i map_dim(512,256);
    sf::RenderWindow window(sf::VideoMode(map_dim.x, map_dim.y), "Tilemap");
    sf::Texture texture;
   // window.setFramerateLimit(60);
    if (!texture.loadFromFile("FFSPRITE.gif"))
    {
        std::cout << "Failed to load player spritesheet!" << std::endl;
        return 1;
    }
    
    
    
    
    bool has_clicked = false;
    
    // set up tshe animations for all four directions (set spritesheet and push frames)
    Animation walkingAnimationDown;
    walkingAnimationDown.setSpriteSheet(texture);
    walkingAnimationDown.addFrame(sf::IntRect(47,5, 16,16)    );
    walkingAnimationDown.addFrame(sf::IntRect(65,5,16, 16)     );
    walkingAnimationDown.addFrame(sf::IntRect(47,5, 16,16)    );
    
    Animation walkingAnimationLeft;
    walkingAnimationLeft.setSpriteSheet(texture);
    walkingAnimationLeft.addFrame(sf::IntRect(83,5, 13,16 ) );
    walkingAnimationLeft.addFrame(sf::IntRect(98,5,14,16));
    walkingAnimationLeft.addFrame(sf::IntRect(83,5,13,16));
    
    
    Animation walkingAnimationRight;
    walkingAnimationRight.setSpriteSheet(texture);
    walkingAnimationRight.addFrame(sf::IntRect(150,5, 13, 16) );
    walkingAnimationRight.addFrame(sf::IntRect(165,5,14, 16)  );
    walkingAnimationRight.addFrame(sf::IntRect(150,5,13,16)   );
    
    Animation walkingAnimationUp;
    walkingAnimationUp.setSpriteSheet(texture);
    walkingAnimationUp.addFrame(sf::IntRect(114, 5, 16, 16));
    walkingAnimationUp.addFrame(sf::IntRect(132, 5, 16, 16));
    walkingAnimationUp.addFrame(sf::IntRect(114, 5, 16, 16));
    
    
    Animation* currentAnimation = &walkingAnimationDown;
    
    // set up AnimatedSprite
    AnimatedSprite animatedSprite(sf::seconds(0.2), true, false);
    
    animatedSprite.setPosition(sf::Vector2f(4.0*32.0,7.0*32.0));
    sf::Clock frameClock;
    
    float speed = 1.f;
    bool noKeyWasPressed = true;
    // define the level with an array of tile indices
    const int level[] =
    {
        0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 1, 1, 1, 3, 3, 3, 3, 0, 0, 0, 2, 0, 0, 0, 0,
        1, 1, 0, 0, 3, 0, 0, 3, 0, 3, 3, 3, 3, 3, 3, 3,
        0, 3, 3, 3, 3, 0, 3, 3, 0, 3, 1, 1, 1, 0, 0, 0,
        0, 3, 1, 0, 2, 2, 3, 0, 0, 3, 1, 1, 1, 2, 0, 0,
        0, 3, 1, 0, 2, 0, 3, 2, 0, 3, 1, 1, 1, 1, 2, 0,
        2, 3, 1, 0, 2, 0, 3, 2, 2, 3, 1, 1, 1, 1, 1, 1,
        0, 3, 3, 3, 3, 2, 3, 3, 3, 3, 0, 0, 1, 1, 1, 1,
    };
    int y_tileSize = 32;
    int x_tileSize = 32;
    vector<vector<int>> main_map;
    for(int i = 0; i < map_dim.y/y_tileSize; i++)
    {
        vector<int> row;
        for(int q = 0; q < map_dim.x/x_tileSize; q++)
            row.push_back(level[i*(map_dim.x/x_tileSize)+ q]);
        main_map.push_back(row);
    }
    
    int xstart = 5;
    int ystart = 8;
    int yFin = 3;
    int xFin = 16;
    int tile_move = 3;
    A_STAR_PATH rest(map_dim.y/y_tileSize,map_dim.x/x_tileSize,main_map,8);
    string route= rest.pathFind(xstart-1,ystart-1, xFin-1, yFin-1, tile_move);
    cout << route << endl;  
    // create the tilemap from the level definition
    TileMap map;
    if (!map.load("tileset.png", sf::Vector2u(32, 32), level, 16, 8))
        return -1;
    unique_ptr<Animation> current = unique_ptr<Animation>(new Animation{*currentAnimation});
    
    unique_ptr<AnimatedSprite> anime = unique_ptr<AnimatedSprite>(new AnimatedSprite{animatedSprite});
    size_t count = 0;
    animatedSprite.play(*currentAnimation);
    // run the main loop
    while (window.isOpen())
    {
        
        sf::Event event;
        char ada;
        int qx;
        if(count < route.length())
        {
            ada = route.at(count);
            qx = atoi(&ada);
            count += 1;
        }
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape)
                window.close();
        }
        
        sf::Time frameTime = frameClock.restart();
        
        // if a key was pressed set the correct animation and move correctly
        sf::Vector2f movement(0.f, 0.f);
        
        
        if (qx == 6)
        {
            currentAnimation = &walkingAnimationUp;
            movement.y -= speed;
            noKeyWasPressed = false;
        }
        
        if (qx == 2)
        {
            currentAnimation = &walkingAnimationDown;
            movement.y += speed;
            noKeyWasPressed = false;
        }
        
        
        if(qx == 4)
        {
            currentAnimation = &walkingAnimationLeft;
            movement.x -= speed;
            noKeyWasPressed = false;
        }
        
        
        if (qx == 0)
        {
            currentAnimation = &walkingAnimationRight;
            movement.x += speed;
            noKeyWasPressed = false;
        }
        if (qx == 1)
        {
            currentAnimation = &walkingAnimationDown;
            movement.y += speed;
            movement.x += speed;
            noKeyWasPressed = false;
        }
        if(qx==3)
        {
            currentAnimation = &walkingAnimationDown;
            movement.y += speed;
            movement.x -= speed;
            noKeyWasPressed = false;
        }
        if(qx == 5)
        {
            currentAnimation = &walkingAnimationUp;
            movement.y -= speed;
            movement.x -= speed;
            noKeyWasPressed = false;
        }
        if(qx == 7)
        {
            currentAnimation = &walkingAnimationUp;
            movement.y -= speed;
            movement.x += speed;
            noKeyWasPressed = false;
        }
       
        for(int i = 0; i < x_tileSize; i++)
        {
            for(int q = 0; q < 3; q++)
            animatedSprite.play(*currentAnimation);
            animatedSprite.move(sf::Vector2f(movement.x,0));
            
        }
        for(int i = 0; i < y_tileSize; i++)
        {
            for(int q = 0; q < 3; q ++ )
                animatedSprite.play(*currentAnimation);
            animatedSprite.move(sf::Vector2f(0,movement.y));
            
            
        }
        std::chrono::milliseconds dura( 200 );
        std::this_thread::sleep_for( dura );
        // if no key was pressed stop the animation
        if (noKeyWasPressed)
        {
            animatedSprite.stop();
        }
        noKeyWasPressed = true;
        animatedSprite.update(frameTime);
        window.clear();
        window.draw(map);
        window.draw(animatedSprite);
        
        window.display();
    }
    
    return 0;
}