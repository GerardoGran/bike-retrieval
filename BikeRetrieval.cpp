#include <cmath>
#include <iostream>
#include <list>
#include <stack>
#include <string>
#include <vector>

using namespace std;

struct sNode
{
    bool bObstacle = false; //Is it an obstruction?
    bool bVisited = false;  //Have we searched it?
    bool bIsPath = false;   //Is it in the final path?
    float fGlobalGoal;      //Distance to goal so far
    float fLocalGoal;       //Distance if we took the alternative
    int x;                  //Node position in 2D space
    int y;
    vector<sNode *> vecNeighbors; //Connections to neighbors
    sNode *parent;                //Node connecting to ths node that offers shortest path so far
};

float heuristic(sNode *a, sNode *b) //Returns Euclidean distance between two nodes
{
    return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

sNode *nodes = nullptr;
stack<sNode *> pathNodes;
int width = 18;
int height = 15;

sNode *nodeStart = nullptr;
sNode *originalStart = nullptr;
sNode *nodeEnd = nullptr;
sNode *bikeNode = nullptr;

vector<string> pathString;
stack<string> path;
float time = 0.0f;

void Solve_AStar()
{
    for (int x = 0; x < width; ++x)
        for (int y = 0; y < height; ++y)
        {
            nodes[y * width + x].fGlobalGoal = INFINITY;
            nodes[y * width + x].fLocalGoal = INFINITY;
            nodes[y * width + x].parent = nullptr;
            nodes[y * width + x].bVisited = false;
        }

    sNode *nodeCurrent = nodeStart;
    nodeStart->fLocalGoal = 0.0f;
    nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

    list<sNode *> listNotTestedNodes;
    listNotTestedNodes.push_back(nodeStart);

    while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)
    {
        //Sort Untested nodes by Global Goal, so lowest is first
        listNotTestedNodes.sort([](const sNode *lhs, const sNode *rhs) { return lhs->fGlobalGoal < rhs->fGlobalGoal; });

        while (!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
            listNotTestedNodes.pop_front();

        if (listNotTestedNodes.empty())
            break;

        nodeCurrent = listNotTestedNodes.front();
        nodeCurrent->bVisited = true;

        for (auto nodeNeighbor : nodeCurrent->vecNeighbors)
        {
            if (!nodeNeighbor->bVisited && !nodeNeighbor->bObstacle)
                listNotTestedNodes.push_back(nodeNeighbor);

            float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + heuristic(nodeCurrent, nodeNeighbor);

            if (fPossiblyLowerGoal < nodeNeighbor->fLocalGoal)
            {
                nodeNeighbor->parent = nodeCurrent;
                nodeNeighbor->fLocalGoal = fPossiblyLowerGoal;

                nodeNeighbor->fGlobalGoal = nodeNeighbor->fLocalGoal + heuristic(nodeNeighbor, nodeEnd);
            }
        }
    }
}

void setNeighbors()
{
}

int main()
{
    nodes = new sNode[width * height];
    stack<sNode *> pathNodes; //Vector for storing the nodes in the shortest path

    //Fills Nodes;
    for (int x = 0; x < width; ++x)
        for (int y = 0; y < height; ++y)
        {
            nodes[y * width + x].x = x;
            nodes[y * width + x].y = y;
            nodes[y * width + x].bObstacle = false;
            nodes[y * width + x].parent = nullptr;
            nodes[y * width + x].bVisited = false;
            nodes[y * width + x].bVisited = false;
        }

    //Fills obstacles;
    nodes[7 * width + 7].bObstacle = true;
    nodes[8 * width + 7].bObstacle = true;
    nodes[7 * width + 10].bObstacle = true;
    nodes[8 * width + 10].bObstacle = true;
    for (int x = 13; x < width; ++x)
        for (int y = 7; y < 9; ++y)
            nodes[y * width + x].bObstacle = true;

    //Set Connections
    for (int x = 0; x < width; ++x)
        for (int y = 0; y < height; ++y)
        {
            if (y > 0) //North Neighbor //If not on the top
                nodes[y * width + x].vecNeighbors.push_back(&nodes[(y - 1) * width + (x)]);
            if (y < height - 1) //South Neighbor //If not on the bottom
                nodes[y * width + x].vecNeighbors.push_back(&nodes[(y + 1) * width + (x)]);
            if (x > 0) //West Neighbor //If not at the left
                nodes[y * width + x].vecNeighbors.push_back(&nodes[(y)*width + (x - 1)]);
            if (x < width - 1 && !(x == 6 && y == 9)) //East Neighbor //If not at the right
                nodes[y * width + x].vecNeighbors.push_back(&nodes[(y)*width + (x + 1)]);
        }

    //Prints parking lot
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (nodes[y * width + x].bObstacle)
                cout << "[ X ]";
            else if (nodes[y * width + x].bIsPath)
                cout << "[ O ]";
            else
                cout << "[" << x << "," << y << "]";
        }
        cout << endl;
    }

    int nodeX, nodeY;
    cout << "Choose bike position in x: ";
    cin >> nodeX;
    cout << "Choose bike position in y: ";
    cin >> nodeY;

    //Select Node Start and Node End
    if (nodeX < 9)
        nodeStart = &nodes[1];
    else
        nodeStart = &nodes[250];
    originalStart = nodeStart;
    nodeEnd = &nodes[nodeY * width + nodeX];

    Solve_AStar();

    //Store path by starting at the end, following the node trail
    //Walk Back through path of pt 1. and store it in path_nodes
    if (nodeEnd != nullptr)
    {
        sNode *p = nodeEnd;
        while (p->parent != nullptr)
        {
            //Prepare For Mini Astar()
            /*
            if (p->parent->parent != nullptr)
            {
                p->parent->bObstacle = true;
                nodeStart = p;
                nodeEnd = p->parent->parent;
                Solve_AStar();
            }
            */

            pathNodes.push(p->parent);
            p->bIsPath = true;

            time += (p->x - p->parent->x) * 1.875 + (p->y - p->parent->y) * 1.14; //Adds time necessary for moving platforms

            path.push("-> [" + to_string(p->x) + "," + to_string(p->y) + "]");
            pathString.push_back(to_string(p->x) + "," + to_string(p->y) + ";");

            //Set next node to this node's parent
            p = p->parent;
        }
        path.push("[" + to_string(originalStart->x) + "," + to_string(originalStart->y) + "]");
        pathString.push_back(to_string(originalStart->x) + "," + to_string(originalStart->y) + ";");
    }

    //Prints parking lot
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (&nodes[y * width + x] == nodeStart)
                cout << "[S]";
            else if (&nodes[y * width + x] == nodeEnd)
                cout << "[E]";
            else if (nodes[y * width + x].bObstacle)
                cout << "[X]";
            else if (nodes[y * width + x].bIsPath)
                cout << "[O]";
            else if (nodes[y * width + x].bVisited)
                cout << "[-]";
            else
                cout << "[ ]";
        }
        cout << endl;
    }

    //Print Path coords
    while (!path.empty())
    {
        cout << path.top();
        path.pop();
    }
    cout << endl
         << "Total time needed for pt 1: " << time << " seconds";
    return 0;
}