#include <vector>
#include <random>
#include <unordered_set>
#include <set>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <fstream>
#include <queue>

using namespace std;

//Assumptions:
// - People choose the shortest route (not the fastest)
// - Only cars are on the roads, public transportation has no effect
// - No carpooling
// - No stoplights
// - One trip per day (go to work in the morning, back in the evening)
// - Time to transverse edge is a function of cars on the road
// - No lanes

double distance(double lat1, double lon1, double lat2, double lon2) {
    //haversine distance formula

    // delta latitude and delta longitude (converted to radians)
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    // convert lat1 and lat2 themselves to radians
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;

    //apply haversine formula for spherical surface distance
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double earth_radius = 6371000; //in m
    double c = 2 * asin(sqrt(a));
    return earth_radius * c;
}


struct Edge {
    int person_time = 0;
    int start_node;
    int end_node;
    double edge_length = 200.0; //Euclidean distance between start and end node
    double max_speed = 4.47;
    int num_drivers = 0;
    int lanes;
    //unordered_set<Driver*> drivers = unordered_set<Driver*>();
    
    Edge(int start_node, int end_node, double length, int lanes = 1, double speedLimit = 4.47)
        :start_node(start_node), end_node(end_node), edge_length(length), lanes(lanes), max_speed(speedLimit) {}

    double add_driver() {
        num_drivers++;
        //drivers.insert(driver);
        return calculate_time();
    }
    
    void remove_driver() {
        num_drivers--;
    }

    void update() {
        person_time += num_drivers;
    }

    double calculate_time() {
        //calculation formula based on Underwood model
        // speed: V_max*e^(-D/D_max)
        // Underwood, R. T. 1961. Speed, volume and density relationship, in B. D. Greenshields, H. P. George, N. S. Guerin, M.R. Palmer, R. T. Underwood (Eds.). Quality and Theory of Traffic Flow: a Symposium. Bureau Highway Traffic, Yale University, 141–188
        double speed = max_speed * pow(M_E, num_drivers/lanes/edge_length/0.25);
        double time = edge_length / speed;
        return time;
    }
};

// adjacency list representation of the graph
class Graph {
    public:
    unordered_map<long long, pair<int, pair<double, double>>> id_map; //database id -> internal id, geographic location (x,y)
    unordered_map<int, pair<double, double>> locations; //internal id -> geographic location
    vector<vector<Edge>> adj_list = {}; // List of all neighbor nodes 

    // Construction of graph from the parsed csv file's data
    // Some error handling required where data is missing or non-standard
    Graph(vector<vector<string>> &nodesData, vector<vector<string>> &edgesData) {
        int index = 0;
        for(int i = 0; i < nodesData.size()-1; ++i) {
            //i+1 because first row is column headers
            auto location = make_pair(stod(nodesData[i+1][2]),stod(nodesData[i+1][1]));
            id_map.insert(make_pair(stoll(nodesData[i+1][0]), make_pair(index, location)));
            locations.insert(make_pair(index, location));
            index++;
        }
        adj_list.resize(id_map.size());
        for(int i = 1; i < edgesData.size(); ++i) {
            auto &edge = edgesData[i];
            if(id_map.find(stoll(edge[0])) != id_map.end() && id_map.find(stoll(edge[1])) != id_map.end()) {
                auto startNode = id_map[stoll(edge[0])];
                auto endNode = id_map[stoll(edge[1])];
                double speedLimit = 15;
                int lanes = 1;
                //some entries don't have speed limits recorded
                if(!edge[7].empty()) {
                    if(edge[7][0]=='[') {
                        if(edge[7][2]!='u') { // some are labeled 'unposted'
                            speedLimit = stoi(edge[7].substr(2,2));
                        }
                    } else if(edge[7][0]!='u') { // some are labeled 'unposted'
                        speedLimit = stoi(edge[7].substr(0,2));
                    }
                }
                if(!edge[6].empty()) {
                    if(edge[6][0]=='[') {
                        lanes = stoi(edge[6].substr(2,1));
                    } else {
                        lanes = stoi(edge[6]);
                    }
                }
                adj_list[startNode.first].push_back(Edge(startNode.first, endNode.first, stod(edge[12]),lanes,speedLimit*0.447));
                if(edge[9]=="TRUE") {
                    adj_list[endNode.first].push_back(Edge(endNode.first, startNode.first, stod(edge[12]), lanes, speedLimit*0.447));
                }
            }
        }
    }

    
    vector<Edge*> shortestPath(int start, int end) {
        if(start==end) {
            return {};
        }
        // A* search
        // Distance heuristic: (Real distance to next node) + (Crow-fly distance from next to end node)

        // heap for efficient processing of next smallest-weight node
        // using existent "vector<pair<double, pair<int, Edge*>>>" for efficient construction
        priority_queue<pair<double, pair<int, Edge*>>, vector<pair<double, pair<int, Edge*>>>, greater<pair<double, pair<int, Edge*>>>> possibilities; // estimated cost, node, previous edge
        Edge null_edge = Edge(0,0,0,0); //dummy edge for default (makes debugging easier)
        //vector of searched edges for path reconstruction; -1 / nulll edge refer to terminal conditions
        vector<pair<double, Edge*>> visited(adj_list.size(), make_pair(-1, &null_edge)); // node -> actual cost, previous edge
        visited[start] = make_pair(0.0, &null_edge); // initialize start node with 0 cost and prev_edge->null
        pair<int, pair<double, Edge*>> answer = make_pair(-1, make_pair(0, &null_edge));


        auto end_loc = locations[end];
        //hard-running once for the first node, then again until possibilities-heap becomes empty
        for(auto &edge: adj_list[start]) {
            // estimated_cost = real_cost + heuristic_cost
            // real_cost = distance to neighbor edge
            // heuristic_cost = straight-line distance from neighbor to end
            auto node_loc = locations[edge.end_node]; // edge to each neighbor
            double real_cost = edge.edge_length;
            double heuristic_cost = distance(node_loc.first, node_loc.second, end_loc.first, end_loc.second);
            double estimated_cost = heuristic_cost + real_cost;
            possibilities.push(make_pair(estimated_cost, make_pair(edge.end_node, &edge)));
        }

        while(!possibilities.empty()) { // Haven't reached the end yet
            auto selected = possibilities.top();
            possibilities.pop();
            auto prev_edge = selected.second.second;
            //convert to searched format
            auto searching = make_pair(selected.second.first, make_pair(visited[prev_edge->start_node].first+prev_edge->edge_length, prev_edge));
            if(searching.first==end) {
                answer = searching;
                break;
            }
            // add neighbors to heap using estimated distance
            for(auto &edge: adj_list[searching.first]) {
                auto node_loc = locations[edge.end_node];
                auto estimated_cost = searching.second.first + edge.edge_length + distance(node_loc.first, node_loc.second, end_loc.first, end_loc.second);
                if(visited[edge.end_node].first < 0) { //means node unvisited
                    possibilities.push(make_pair(estimated_cost, make_pair(edge.end_node, &edge)));
                }
            }
            visited[searching.first] = searching.second;

        }
        if(answer.first==-1) {
            return {}; //didn't find a path from start to end
        }
        // Save it
        vector<Edge*> return_value = {answer.second.second};
        auto current_node = answer.second.second->start_node;
        while(current_node != start) {
            auto next_edge = visited[current_node].second;
            return_value.push_back(next_edge);
            current_node = next_edge->start_node;
        }
        // Reverse it
        for(int i = 0; i < return_value.size() / 2; ++i) {
            auto temp = return_value[i];
            return_value[i] = return_value[return_value.size()-i-1];
            return_value[return_value.size()-i-1] = temp;
        }
        return return_value;
    }
};

class Driver {
    int home;
    int work;
    int leave_home; // time leaves home
    int return_home; // time returns home
    bool outward = true; // true represents going to work, false represents returning home
    int current_edge = -1; //an index on path
    int edge_length = 0; //in seconds
    int time_waited = 0; //time waiting on the current edge
    vector<Edge*> out_path; //path to work
    vector<Edge*> in_path; //path to home

    public:
    int transit_time = 0;
    bool failed = false;

    private:
    bool next_edge() { // returns if reached destination
        time_waited -= edge_length;
        transit_time += edge_length;
        if(outward) {
            if(current_edge >= 0)
                out_path[current_edge]->remove_driver();
            current_edge++;
            if(current_edge >= out_path.size()) {
                time_waited = 0;
                outward = false;
                edge_length = 0;
                current_edge = -1;
                return true;
            }
            edge_length = (int) out_path[current_edge]->add_driver();
        } else {
            if(current_edge >= 0)
                in_path[current_edge]->remove_driver();
            current_edge++;
            if(current_edge >= in_path.size()) {
                time_waited = 0;
                outward = true;
                edge_length = 0;
                current_edge = -1;
                return true;
            }
            edge_length = (int)in_path[current_edge]->add_driver();
        }
        return false;
    }

    public:
        Driver(int inHome, int inWork, Graph* graph) {
            // populate graph method will create a distribution closely equal to Baltimore's neighborhoods
            home = inHome;
            //home = rand() % totalNodes;
            work = inWork;
            //work = rand() % totalNodes;
            out_path = graph->shortestPath(home, work);
            in_path = graph->shortestPath(work, home);
            if(out_path.size()==0 || in_path.size()==0) {
                failed = true;
            }
            // seconds since midnight
            leave_home = 21600; //represents 6:00, should sample from a distribution
            return_home = 61200; //represents 17:00, should sample from a distibution
        }

        void update(int wall_time, int delta_time = 1) { //updates secs seconds (lower = better precision)
            if(outward && wall_time >= leave_home && wall_time < return_home) {
                time_waited += delta_time;
                if(time_waited >= edge_length) next_edge();
            } else if(!outward && wall_time > return_home) {
                time_waited += delta_time;
                if(time_waited >= edge_length) next_edge();
            }
        }
};

class Simulation {
    int num_drivers;
    int time = 0;
    vector<Driver> drivers;
    Graph graph;

    void populate_graph(vector<pair<pair<double,double>,int>> pop_info) {
        // - Takes input which is a vector of neighborhood population info
        //   neighborhood coordinates (pair<double, double>) -> population size (int).
        // - Make a set of all nodes within a 2 km radius of the coordinates
        // - Randomly assign n (n=pop_size) drivers to nodes in the area
        
        // go over each known neighborhood or "area"
        for (auto area : pop_info) {
            double area_lat = area.first.first;
            double area_lon = area.first.second;
            int area_population = area.second;

            // create a list of all nodes within a 2 km radius of area's coords
            vector<int> nodes_in_area;
            for (auto node : graph.locations) {
                pair<double, double> coords = node.second;                
                if (distance(area_lat, area_lon, coords.first, coords.second) <= 2) {
                    nodes_in_area.push_back(node.first); // add node's id to the list
                }
            }

            // add pop_size number of people to random nodes in the area
            for (int i=0; i<area_population; i++) {
                //TODO : Find efficient way to get random node from graph.locations
                Driver new_driver = Driver(
                    nodes_in_area[ rand() % nodes_in_area.size() ],
                    rand() % graph.locations.size(),
                    &graph);
                drivers.push_back(new_driver);
            }
        }
    }

    public:
        Simulation(int num_drivers, Graph&& graph): num_drivers(num_drivers), graph(graph) {
            drivers.reserve(num_drivers);
            for(int i = 0; i < num_drivers; ++i) {
                drivers.push_back(Driver(rand() % graph.adj_list.size(), rand() % graph.adj_list.size(), &graph));
                if(drivers.back().failed) {
                    drivers.pop_back();
                }
            }
            time = 0;
        }
        void run() {

            for(int i = 21540; i < 64800; i++) { // starts at 5:59 am and ends at 6:00 pm
                for(auto &driver: drivers) {
                    driver.update(i, 1);
                }
                for(auto &node: graph.adj_list) {
                    for(auto &edge: node) {
                        edge.update();
                    }
                }
            }
            double time = 0;
            for(auto const &driver: drivers) {
                time += (double) driver.transit_time;
            }
            time /= drivers.size();
            cout << "Average time: " << time;
        }
};

vector<vector<string>> parse_csv(ifstream &file) {
    vector<vector<string>> all_data;
    for(string line; !getline(file, line).eof();) {
        vector<string> row;
        int prev_index = 0;
        int comma_index = line.find(",");
        while (comma_index != string::npos) { 
            if (line[prev_index] =='"') { //inside quotes -> exclude the quotes
                comma_index = line.find('"', prev_index+1);
                // Ignore "" because that is the escape character for a " (we don't) use this data
                // so it is ok if it remains doubled
                while(comma_index < line.size()-1 && line[comma_index+1] != ',') {
                    comma_index = line.find('"', comma_index+2);
                }
                row.push_back(line.substr(prev_index+1, comma_index - prev_index));
                prev_index = comma_index + 2;
            }
            else { //not inside quotes
                row.push_back(line.substr(prev_index, comma_index - prev_index));
                prev_index = comma_index + 1;
            }
            
            comma_index = line.find(",", prev_index);
        }
        
        all_data.push_back(row);
    }
    return all_data;
}

int main() {
    srand(15);
    const string edges_file = "edges_drive.csv";
    const string nodes_file = "nodes_drive.csv";
    ifstream edges_csv = ifstream(edges_file);
    ifstream nodes_csv = ifstream(nodes_file);
    vector<vector<string>> node_data = parse_csv(nodes_csv);
    vector<vector<string>> edge_data = parse_csv(edges_csv);
    auto sim = Simulation(50, Graph(node_data, edge_data));
    sim.run();
}
