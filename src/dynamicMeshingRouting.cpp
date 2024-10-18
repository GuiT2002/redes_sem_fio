#include "DynamicMeshingRouting.h"

uint32_t receiverNode = 0;

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
}

void DynamicMeshingRouting::loop() {
    mesh.update();
}

void DynamicMeshingRouting::sendMessage(uint32_t nodeId) {
    // Prepare the message to be broadcasted
    String msg = "Hello from Node " + String(nodeId);

    // Send the message to all nodes (broadcast)
    mesh.sendBroadcast(msg);
    Serial.printf("Node %u broadcasting message\n", nodeId);
}


void DynamicMeshingRouting::receivedCallback(uint32_t from, String &msg) {
    Serial.printf("Received from node %u: %s\n", from, msg.c_str());

    // Check if this node is the receiver
    if (nodeRoles[mesh.getNodeId()] == "receiver") {
        Serial.println("Message received at the receiver: " + msg);
    } else {
        // If this is a forwarder, append this node's ID to the path and forward the message
        String newPath = msg + " -> " + String(mesh.getNodeId());
        uint32_t nextNode = findBestPath(mesh.getNodeId(), receiverNode);  // Find next hop to the receiver
        if (nextNode != -1) {
            mesh.sendSingle(nextNode, newPath);
            Serial.printf("Forwarding message to node %u\n", nextNode);
        } else {
            Serial.println("No available path.");
        }
    }
}


void DynamicMeshingRouting::newConnectionCallback(uint32_t nodeId) {
    Serial.printf("New connection with node %u\n", nodeId);
    initializeNodeCosts(nodeId);

    // Dynamically assign roles if needed
    if (nodeRoles.empty()) {
        nodeRoles[nodeId] = "receiver";  // First node becomes the receiver (central node)
    } else {
        nodeRoles[nodeId] = "sender";  // Other nodes become senders
    }

    recalculatePaths();  // Recalculate paths whenever a new node connects
}

void DynamicMeshingRouting::droppedConnectionCallback(uint32_t nodeId) {
    Serial.printf("Node %u disconnected\n", nodeId);
    removeNodeCosts(nodeId);

    // Remove node role
    nodeRoles.erase(nodeId);

    recalculatePaths();  // Recalculate paths when a node disconnects
}

uint32_t DynamicMeshingRouting::findBestPath(uint32_t from, uint32_t to) {
    std::map<uint32_t, int> distances;
    std::map<uint32_t, uint32_t> previous;
    std::vector<uint32_t> unvisited;

    for (const auto &node : mesh.getNodeList()) {
        distances[node] = (node == from) ? 0 : INT_MAX;
        unvisited.push_back(node);
    }

    while (!unvisited.empty()) {
        uint32_t currentNode = unvisited.front();
        for (auto node : unvisited) {
            if (distances[node] < distances[currentNode]) {
                currentNode = node;
            }
        }

        unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), currentNode), unvisited.end());

        if (currentNode == to) {
            paths[to].push_back(previous[to]);
            return currentNode;
        }

        for (const auto &neighbor : costs[currentNode]) {
            int newDist = distances[currentNode] + neighbor.second; 
            if (newDist < distances[neighbor.first]) {
                distances[neighbor.first] = newDist;
                previous[neighbor.first] = currentNode;
            }
        }
    }

    return -1;
}

void DynamicMeshingRouting::increaseCost(uint32_t from, uint32_t to) {
    costs[from][to] += random(1, 5);
    costs[to][from] = costs[from][to];
}

void DynamicMeshingRouting::initializeNodeCosts(uint32_t newNode) {
    for (auto &entry : costs) {
        costs[newNode][entry.first] = random(5, 15);
        costs[entry.first][newNode] = costs[newNode][entry.first];
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
