// Created by Leonardo Romanini de Barros, Joseph Sinder, and Vince Schifano

#ifndef GRAPH_H
#define GRAPH_H
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <math.h>
#include<queue>
using namespace std;

class Graph {
public:
    map<long, pair<double,double>> nodes;
    unordered_map<long, vector<pair<long, double>>> edges;

public:
    void insertNode(long id, double lat, double lon) {
        nodes[id] = make_pair(lat, lon);

    }

    void insertEdge(long id1, long id2) {
        double weight;

        double lat1 = nodes[id1].first;
        double lon1 = nodes[id1].second;

        double lat2 = nodes[id2].first;
        double lon2 = nodes[id2].second;

        weight = sqrt(pow(lat1-lat2,2) + pow(lon1-lon2,2));

        edges[id1].push_back(make_pair(id2,weight));
        edges[id2].push_back(make_pair(id1,weight));
    }

    double dijkstra(long start, long end) {
        std::map<long, double> dist;
        std::map<long, long> prev;

        for (const auto& [node, _] : nodes) {
            dist[node] = INFINITY;
        }

        dist[start] = 0.0;
        priority_queue<pair<double, long>, vector<pair<double, long>>, greater<>> pq;
        pq.push({0.0, start});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            if (u == end) break;

            for (const auto& [v, weight] : edges[u]) {
                if (dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        if (dist[end] == INFINITY) {
            cout << "No path exists from " << start << " to " << end << endl;
            return -1;
        }

        vector<long> path;
        double distanceTraversed = 0.0;

        for (long at = end; at != start; at = prev[at]) {
            long prevNode = prev[at];
            path.push_back(at);
            for (const auto& [neighbor, weight] : edges[prevNode]) {
                if (neighbor == at) {
                    distanceTraversed += weight;
                    break;
                }
            }
        }
        path.push_back(start);
        reverse(path.begin(), path.end());



        cout << "Dijkstra path: ";
        for (long id : path) {
            cout << id << " (" << nodes[id].first << ", " << nodes[id].second << "), ";
        }
        cout << "\n";

        cout << "Total distance traveled: " << distanceTraversed*362776.86915 << " feet" << endl;


        return dist[end];
    }

double heuristic(long id1, long id2) {
    double lat1 = nodes[id1].first;
    double lon1 = nodes[id1].second;
    double lat2 = nodes[id2].first;
    double lon2 = nodes[id2].second;
    return sqrt(pow(lat1 - lat2, 2) + pow(lon1 - lon2, 2));
}

double aStar(long start, long end) {
    if (nodes.find(start) == nodes.end() || nodes.find(end) == nodes.end()) {
        cout << "Invalid Node Entries." << endl;
        return -1;
    }

    unordered_map<long, double> gScore;
    unordered_map<long, double> fScore;
    unordered_map<long, long> cameFrom;

    for (const auto& [node, _] : nodes) {
        gScore[node] = INFINITY;
        fScore[node] = INFINITY;
    }

    gScore[start] = 0.0;
    fScore[start] = heuristic(start, end);

    priority_queue<pair<double, long>, vector<pair<double, long>>, greater<>> openSet;
    openSet.push({fScore[start], start});

    while (!openSet.empty()) {
        long current = openSet.top().second;
        openSet.pop();

        if (current == end) {
            vector<long> path;
            double pathWeightSum = 0.0;
            long currentNode = end;

            while (cameFrom.find(currentNode) != cameFrom.end()) {
                long prevNode = cameFrom[currentNode];
                path.push_back(currentNode);

                for (const auto& [neighbor, weight] : edges[prevNode]) {
                    if (neighbor == currentNode) {
                        pathWeightSum += weight;
                        break;
                    }
                }

                currentNode = prevNode;
            }
            path.push_back(start);
            reverse(path.begin(), path.end());

            cout << "A* path: ";
            for (long id : path) {
                cout << id << " (" << nodes[id].first << ", " << nodes[id].second << "), ";
            }
            cout << "\nTotal distance traveled: " << pathWeightSum*362776.86915 << " feet" << endl;

            return gScore[end];
        }

        for (auto [neighbor, weight] : edges[current]) {
            double tentative_gScore = gScore[current] + weight;
            if (tentative_gScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, end);
                openSet.push({fScore[neighbor], neighbor});
            }
        }
    }

    cout << "No path exists from " << start << " to " << end << endl;
    return -1;
}



};




#endif
