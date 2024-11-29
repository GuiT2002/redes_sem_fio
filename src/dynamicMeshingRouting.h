#ifndef DYNAMIC_MESHING_ROUTING_H
#define DYNAMIC_MESHING_ROUTING_H

#include "painlessMesh.h"
#include <map>
#include <vector>
#include <queue>       // for priority_queue
#include <utility>     // for pair

#define MESH_PREFIX     "meshNetwork"
#define MESH_PASSWORD   "securePassword"
#define MESH_PORT       5555

class DynamicMeshingRouting {
public:
    DynamicMeshingRouting();
    void setup();  // Setup without fixed ID
    void loop();

private:
    Scheduler userScheduler;
    painlessMesh mesh;
    Task taskSendMessage;

    std::map<uint32_t, std::map<uint32_t, int>> costs; // Custo das arestas
    std::map<uint32_t, std::vector<uint32_t>> paths; // Caminhos percorridos
    std::map<uint32_t, String> nodeRoles; // Mapeamento dos papéis dos nós

    void sendMessage(uint32_t nodeId);
    void receivedCallback(uint32_t from, String &msg);
    void newConnectionCallback(uint32_t nodeId);
    void droppedConnectionCallback(uint32_t nodeId);
    uint32_t findBestPath(uint32_t from, uint32_t to);  // Implementação do algoritmo de Dijkstra
    void recalculatePaths(); // Recalcula os caminhos entre os nós
    void increaseCost(uint32_t from, uint32_t to);
    void initializeNodeCosts(uint32_t newNode);
    void removeNodeCosts(uint32_t nodeId);
    void printPath(uint32_t from, uint32_t to); // Método para imprimir o caminho percorrido
    //void printCostMatrix();
};

#endif // DYNAMIC_MESHING_ROUTING_H