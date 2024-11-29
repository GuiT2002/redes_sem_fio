#include "DynamicMeshingRouting.h"

uint32_t receiverNode = 761895598;

DynamicMeshingRouting::DynamicMeshingRouting() 
    : taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, [this](){ sendMessage(mesh.getNodeId()); }) {}

void DynamicMeshingRouting::setup() {
    Serial.begin(115200);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive([this](uint32_t from, String &msg){ receivedCallback(from, msg); });
    mesh.onNewConnection([this](uint32_t nodeId){ newConnectionCallback(nodeId); });
    mesh.onDroppedConnection([this](uint32_t nodeId){ droppedConnectionCallback(nodeId); });

    userScheduler.addTask(taskSendMessage);
    taskSendMessage.enable();

    if (mesh.getNodeId() != receiverNode) {
        taskSendMessage.enable();  // Only enable sending for non-receiver nodes
    }
}

void DynamicMeshingRouting::loop() {
    mesh.update();
}

void DynamicMeshingRouting::sendMessage(uint32_t nodeId) {
    // Only non-receiver nodes should broadcast messages
    if (nodeId == receiverNode) {
        return;
    }

    String msg = "Hello from Node " + String(nodeId);
    mesh.sendBroadcast(msg);
}


void DynamicMeshingRouting::receivedCallback(uint32_t from, String &msg) {
    Serial.printf("[node %u] %s\n", from, msg.c_str());

    if (mesh.getNodeId() == receiverNode) {
        // If this is the receiver, print the full path
        Serial.println("Path: " + msg.substring(16));
        Serial.println();
    } else {
        // If this is a forwarder, append this node's ID to the path and forward the message
        String newPath = msg + " -> " + String(mesh.getNodeId());
        uint32_t nextNode = findBestPath(mesh.getNodeId(), receiverNode);  // Find next hop to the receiver
        if (nextNode != -1) {
            mesh.sendSingle(nextNode, newPath);}
        /* } else {
            Serial.println("No available path.");
        } */
    }

    //printCostMatrix();

    Serial.println();
}


void DynamicMeshingRouting::newConnectionCallback(uint32_t nodeId) {
    Serial.printf("New connection with node %u", nodeId);
    initializeNodeCosts(nodeId);

    // Assign the base station role to the receiver node only
    if (nodeId == receiverNode) {
        nodeRoles[nodeId] = "receiver";
    } else {
        nodeRoles[nodeId] = "sender";
    }

    recalculatePaths();  
}

void DynamicMeshingRouting::droppedConnectionCallback(uint32_t nodeId) {
    //Serial.printf("Node %u disconnected\n", nodeId);
    removeNodeCosts(nodeId);

    // Remove node role
    nodeRoles.erase(nodeId);

    recalculatePaths();  
}

uint32_t DynamicMeshingRouting::findBestPath(uint32_t from, uint32_t to) {
    // Use a priority queue to efficiently select minimum distance node
    std::priority_queue<std::pair<int, uint32_t>, 
                        std::vector<std::pair<int, uint32_t>>, 
                        std::greater<std::pair<int, uint32_t>>> pq;
    
    // Distances map to track shortest known distance to each node
    std::map<uint32_t, int> distances;
    
    // Previous node map to reconstruct the path
    std::map<uint32_t, uint32_t> previous;
    
    // Initialize all distances to infinity except start node
    for (const auto &node : mesh.getNodeList()) {
        distances[node] = INT_MAX;
    }
    distances[from] = 0;
    
    // Push start node to priority queue
    pq.push({0, from});
    
    while (!pq.empty()) {
        // Get node with minimum distance
        int currentDist = pq.top().first;
        uint32_t currentNode = pq.top().second;
        pq.pop();
        
        // If we've reached the destination, reconstruct and return the path
        if (currentNode == to) {
            // Reconstruct path
            std::vector<uint32_t> path;
            while (currentNode != from) {
                path.push_back(currentNode);
                currentNode = previous[currentNode];
            }
            path.push_back(from);
            std::reverse(path.begin(), path.end());
            
            // Store the path
            paths[to] = path;
            
            // Return the destination node
            return to;
        }
        
        // Skip if we've found a better path already
        if (currentDist > distances[currentNode]) continue;
        
        // Check all neighbors
        for (const auto &neighbor : costs[currentNode]) {
            int newDist = distances[currentNode] + neighbor.second;
            
            // If we've found a shorter path
            if (newDist < distances[neighbor.first]) {
                distances[neighbor.first] = newDist;
                previous[neighbor.first] = currentNode;
                pq.push({newDist, neighbor.first});
            }
        }
    }
    
    // No path found
    return -1;
}

void DynamicMeshingRouting::increaseCost(uint32_t from, uint32_t to) {

    if (costs[from][to] > 20){
        costs[from][to] = 1;
        costs[to][from] = 1;
    }
    else{
        costs[from][to] += random(1, 3);
        costs[to][from] += random(1, 3);
    }
}

void DynamicMeshingRouting::initializeNodeCosts(uint32_t newNode) {
    for (auto &entry : costs) {
        costs[newNode][entry.first] = random(1, 10);
        costs[entry.first][newNode] = random(1, 10);
    }
}

void DynamicMeshingRouting::removeNodeCosts(uint32_t nodeId) {
    costs.erase(nodeId);
    for (auto &entry : costs) {
        entry.second.erase(nodeId);
    }
    Serial.printf("Node %u removed from cost matrix\n", nodeId);
}

void DynamicMeshingRouting::recalculatePaths() {
    for (const auto &startNode : mesh.getNodeList()) {
        for (const auto &endNode : mesh.getNodeList()) {
            if (startNode != endNode) {
                findBestPath(startNode, endNode);
            }
        }
    }
}

void DynamicMeshingRouting::printPath(uint32_t from, uint32_t to) {
    std::vector<uint32_t> path;
    uint32_t current = to;

    while (current != from && paths[current].size() > 0) {
        path.push_back(current);
        current = paths[current].back();
    }
    path.push_back(from);

    Serial.print("Path: ");
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        Serial.print(*it);
        if (it + 1 != path.rend()) {
            Serial.print(" -> ");
        }
    }
    Serial.println();
}

/* void DynamicMeshingRouting::printCostMatrix() {
    Serial.println("Cost Matrix:");
    if (costs.empty()) {
        Serial.println("No costs recorded yet.");
        return; // Exit if the costs are empty
    }
    
    for (const auto &fromEntry : costs) {
        for (const auto &toEntry : fromEntry.second) {
            Serial.print("Cost from Node ");
            Serial.print(fromEntry.first);
            Serial.print(" to Node ");
            Serial.print(toEntry.first);
            Serial.print(": ");
            Serial.println(toEntry.second);
        }
    }
    Serial.println(); // Add a blank line for readability
} */