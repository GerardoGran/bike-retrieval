#include <cmath>
#include <iostream>
#include <list>
#include <stack>
#include <string>
#include <vector>
#include <queue>

using namespace std;

struct sNode
{
    bool bCabin = false;
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
vector<sNode *> pathNodes;
int width = 18;
int height = 15;

sNode *nodeStart = nullptr;
sNode *originalStart = nullptr;
sNode *nodeEnd = nullptr;

string finalPathString = "";
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
    //Set Connections
    for (int x = 0; x < width; ++x)
        for (int y = 0; y < height; ++y)
        {
            //North Neighbor //If not on the top
            if (y > 0)
                nodes[y * width + x].vecNeighbors.push_back(&nodes[(y - 1) * width + (x)]);

            //South Neighbor //If not on the bottom
            if (y < height - 1)
                nodes[y * width + x].vecNeighbors.push_back(&nodes[(y + 1) * width + (x)]);

            //West Neighbor //If not at the left
            if (x > 0 && !(x == 8 && y == 9) && !(x == 7 && y == 6) && !(x == 11 && y == 6) && !(x == 10 && y == 9) && !(x == 13 && y == 9))
                nodes[y * width + x].vecNeighbors.push_back(&nodes[(y)*width + (x - 1)]);

            //East Neighbor //If not at the right
            if (x < width - 1 && !(x == 6 && y == 9) && !(x == 7 && y == 6) && !(x == 9 && y == 6) && !(x == 10 && y == 9) && !(x == 13 && y == 9) && !(x == 12 && y == 6))
                nodes[y * width + x].vecNeighbors.push_back(&nodes[(y)*width + (x + 1)]);
        }
}

void fillNodes()
{
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
            nodes[y * width + x].bIsPath = false;
        }
}

void fillObstacles()
{
    //Fills obstacles;
    nodes[7 * width + 7].bObstacle = true;
    nodes[8 * width + 7].bObstacle = true;
    nodes[7 * width + 10].bObstacle = true;
    nodes[8 * width + 10].bObstacle = true;
    nodes[0 * width + 0].bObstacle = true;
    nodes[1 * width + 0].bObstacle = true;
    nodes[13 * width + 17].bObstacle = true;
    nodes[14 * width + 17].bObstacle = true;
    for (int x = 13; x < width; ++x)
        for (int y = 7; y < 9; ++y)
            nodes[y * width + x].bObstacle = true;

    nodes[0 * width + 0].bCabin = true;
    nodes[1 * width + 0].bCabin = true;
    nodes[13 * width + 17].bCabin = true;
    nodes[14 * width + 17].bCabin = true;
}

void printCoords()
{
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
}

vector<sNode *> storePt1()
{
    if (nodeEnd != nullptr)
    {
        sNode *p = nodeEnd;
        while (p->parent != nullptr)
        {
            pathNodes.push_back(p->parent); //Stores backwards path in FIFO to iterate over it for Pt 2
            p->bIsPath = true;              //For Representation in printStep()

            time += (abs(p->parent->x - p->x)) * 1.875 + (abs(p->parent->y - p->y)) * 1.14; //Adds time necessary for moving platforms

            path.push("-> [" + to_string(p->x) + "," + to_string(p->y) + "]");   //for printStep()
            pathString.push_back(to_string(p->x) + "," + to_string(p->y) + ";"); //for Final Representation

            //Set next node to this node's parent
            p = p->parent;
        }
        path.push("-> [" + to_string(nodeStart->x) + "," + to_string(nodeStart->y) + "]");   //for printStep()
        pathString.push_back(to_string(nodeStart->x) + "," + to_string(nodeStart->y) + ";"); //for Final Representation

        while (!path.empty())
        {
            finalPathString.append(path.top());
            path.pop();
        }

        return pathNodes;
    }
}

void storePt2()
{
    if (nodeEnd != nullptr)
    {
        sNode *p = nodeEnd;
        while (p->parent != nullptr)
        {
            p->bIsPath = true; //For Representation in printStep()

            time += (abs(p->parent->x - p->x)) * 1.875 + (abs(p->parent->y - p->y)) * 1.14; //Adds time necessary for moving platforms

            path.push("-> [" + to_string(p->x) + "," + to_string(p->y) + "]");   //for printStep()
            pathString.push_back(to_string(p->x) + "," + to_string(p->y) + ";"); //for Final Representation

            //Set next node to this node's parent
            p = p->parent;
        }

        while (!path.empty())
        {
            finalPathString.append(path.top());
            path.pop();
        }
    }
}

void printStep()
{
    //Prints parking lot
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (&nodes[y * width + x] == nodeStart)
                cout << "[S]";
            else if (&nodes[y * width + x] == nodeEnd)
                cout << "[E]";
            else if (nodes[y * width + x].bCabin)
                cout << "[*]";
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
         << "Total time needed for pt 1: " << time << " seconds" << endl;
}

int main()
{
    nodes = new sNode[width * height];
    vector<sNode *> pathNodes; //Vector for storing the nodes in the shortest path

    fillNodes();

    fillObstacles();

    setNeighbors();

    printCoords();

    //TODO: Add Random Option
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

    Solve_AStar(); //Solves Pt1

    //Store path by starting at the end, following the node trail
    //Walk Back through path of pt 1. and store it in path_nodes
    pathNodes = storePt1();

    // cin.get();
    // printStep();

    //BEGINNING OF PT.2

    /*
    // Resets nodes
    fillNodes();
    fillObstacles();
    setNeighbors();
    */

    // cin.get();
    for (int i = 0; i < pathNodes.size() - 1; ++i)
    {
        sNode *bike = pathNodes.at(i);
        if (bike != originalStart)
        {
            nodeStart = nodeEnd;
            bike->bObstacle = true;
            nodeEnd = pathNodes.at(i + 1);

            Solve_AStar();

            storePt2();
            // printStep();

            bike->bObstacle = false;

            nodeStart = nodeEnd;
            nodeEnd = bike;

            Solve_AStar();
            storePt2();
        }
    }

    printStep();
    cout << finalPathString;

    return 0;
}