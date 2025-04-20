#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include "Graph.h"
#include <iomanip>
using namespace std;

int main()
{
    Graph graph;
    string line;


    //map<long, pair<double,double>> map;
    ifstream input("/Users/leo/Desktop/nodes.txt");
    while(getline(input, line)) {
        stringstream l(line);
        string id;
        string lat;
        string lon;

        l >> (id) >> lat >> lon;
        long id2 = stol(id);
        double lat2 = stod(lat);
        double lon2 = stod(lon);
        graph.insertNode(id2, lat2, lon2);
    }

    unordered_map<long, vector<pair<long, double>>> edges;
    //          id from,        id to, weight

    ifstream inputNode("/Users/leo/Desktop/ConnectedPaths.txt");

    while(getline(inputNode, line)) {
        stringstream l (line);
        string num2;
        string first2;
        string second2;
        int i=1;
        l>>num2;
        int num = stoi(num2);
        long first;
        long second;
         l>>first2;
        first = stol(first2);

        while(i<num) {
            l>>second2;
            second = stol(second2);
            graph.insertEdge(first, second);
            first = second;
            i++;
        }
    }
    long startID;
    long endID;
    cout << "What is your starting ID?" << endl;
    cin >> startID;
    cout << "What is your ending ID?" << endl;
    cin >> endID;


    auto start_time = chrono::high_resolution_clock::now();
    graph.dijkstra(startID, endID);
    auto end_time = chrono::high_resolution_clock::now();

    chrono::duration<double> elapsedDijkstra = (end_time - start_time) * 1000;
    cout << "Dijkstra took " << elapsedDijkstra.count() << " ms.\n" << endl;


    start_time = chrono::high_resolution_clock::now();
    graph.aStar(startID,endID);
    end_time = chrono::high_resolution_clock::now();

    chrono::duration<double> elapsedAstar = (end_time - start_time) * 1000;
    cout << "A* took " << elapsedAstar.count() << " ms.\n" << endl;
    cout << "The A* algorithm computed the fastest path " << setprecision(3) << fixed << elapsedDijkstra / elapsedAstar << " faster than Dijkstra's Algorithm." << endl;


    return 0;
}
