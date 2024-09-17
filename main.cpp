#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "Kinematics/KinematicLibraryPaths.h"



using namespace std;

int main()
{
    vector<string> msg {"Hello", "C++", "World", "from", "VS Code", "and the C++ extension!"};

    for (const string& word : msg)
    {
        cout << word << " ";
    }
    cout << endl;

    Eigen::Vector2d x;
    x << 2, 5;

    cout << x.transpose() << endl;
}
