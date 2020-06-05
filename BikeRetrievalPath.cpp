#include <bits10_1.h>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

float leftBlankHeuristics[15][9];
float rightBlankHeuristics[15][9];

struct cell
{
    int parent_i, parent_j;

    //f = g + h
    float f, g, h;
};

int main()
{
    //Calculate Euclidean distance
    for (int i = 0; i < 15; ++i)
    {
        for (int j = 0; j < 9; ++j)
        {
            leftBlankHeuristics[i][j] = sqrt(pow((1.875 - 1.875 * j), 2) + pow((1.14 * i), 2));
        }
    }

    for (int i = 14; i >= 0; --i)
    {
        for (int j = 8; j >= 0; --j)
        {
            rightBlankHeuristics[i][j] = sqrt(pow((1.875 - 1.875 * (8 - j)), 2) + pow((1.14 - 1.14 * (14 - i)), 2));
        }
    }

    //TEST: Print Matrices
    for (int i = 0; i < 15; ++i)
    {
        for (int j = 0; j < 9; ++j)
        {
            cout << "[" << leftBlankHeuristics[i][j] << "] ";
        }
        cout << endl;
    }

    cout << endl;
    for (int i = 0; i < 15; ++i)
    {
        for (int j = 0; j < 9; ++j)
        {
            cout << "[" << rightBlankHeuristics[i][j] << "] ";
        }
        cout << endl;
    }

    return 0;
}