#ifndef STUDENT_INTERFACE_HPP
#define STUDENT_INTERFACE_HPP
#include <algorithm>
#include <climits>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "studentapi.hpp"

// defined by student
// should probably only need id and path...
class PerCarInfo {
 private:
  car_id_t id;
  std::vector<intersection_id_t> path;

 public:
  PerCarInfo(){};
  PerCarInfo(car_id_t _id) : id(_id){};
  PerCarInfo(car_id_t _id, std::vector<road_id_t> _path) : id(_id), path(_path){};

  car_id_t getID() { return this->id; }

  std::vector<road_id_t> getPath() { return this->path; }

  intersection_id_t getDest() { return this->path.back(); }

  ~PerCarInfo(){};
};

bool compareCarSpeed(std::pair<size_t, size_t> t1, std::pair<size_t, size_t> t2) {
  return t1.first < t2.first;
}

class Road {
 private:
  road_id_t id;
  intersection_id_t from;
  intersection_id_t to;
  size_t length;
  size_t maxCars;
  size_t maxSpeed;
  std::vector<std::pair<size_t, size_t> > carSpeed;

 public:
  Road() {}

  Road(std::vector<int> result) {
    // input the parsed result and initialize each field
    this->id = result[0];
    this->from = result[1];
    this->to = result[2];
    this->length = result[3];
    this->maxCars = result[4];
    this->maxSpeed = result[5];
    for (size_t i = 6; i < result.size(); i += 2) {
      // pair<car, speed>
      carSpeed.push_back(std::make_pair(result[i], result[i + 1]));
    }
    std::sort(carSpeed.begin(), carSpeed.end(), compareCarSpeed);
  }

  std::pair<intersection_id_t, intersection_id_t> getFromTo() {
    return std::make_pair(this->from, this->to);
  }

  road_id_t getID() { return this->id; }

  size_t getLength() { return this->length; }

  size_t getMaxCars() { return this->maxCars; }

  size_t getMaxSpeed() { return this->maxSpeed; }

  size_t calculateSpeed(size_t numCars) {
    // don't need to drop speed
    if (carSpeed.empty() || carSpeed.front().first > numCars) {
      return maxSpeed;
    }
    for (size_t i = 0; i < this->carSpeed.size() - 1; i++) {
      if (carSpeed[i].first <= numCars && carSpeed[i + 1].first > numCars) {
        return carSpeed[i].second;
      }
    }
    if (carSpeed.back().first < maxCars && numCars > maxCars) {
      return 1;
    }
    return carSpeed.back().second;
  }

  size_t getTime() {
    RoadStatus s = query_road(this->id);
    size_t speed = calculateSpeed(s.num_cars);
    if (length % speed) {
      // ceiling division
      return length / speed + 1;
    }
    return length / speed;
  }

  void printRoad() {
    std::cout << "road " << id << ": from " << from << " to " << to
              << ", length=" << length << ";  ";
    for (auto cp : carSpeed) {
      std::cout << "<car: " << cp.first << ", speed: " << cp.second << "> --- ";
    }
    std::cout << "\n";
  }
};

struct NodeVal {
  // previous node, use to backtrack the path
  intersection_id_t prev;
  // the distance from start node to current node
  size_t distance;
  // variable used in Dijkstras
  bool isMarked;
};

// defined by student
// for now, this class is an alias for our GenericGraph
class Graph {
 private:
  std::unordered_map<intersection_id_t, std::unordered_map<intersection_id_t, Road *> >
      nodes;

 public:
  Graph() {}

  std::unordered_map<intersection_id_t, std::unordered_map<intersection_id_t, Road *> >
  getNodes() {
    return nodes;
  }

  void addRoad(Road * road) {
    intersection_id_t from = road->getFromTo().first;
    intersection_id_t to = road->getFromTo().second;
    // add road
    nodes[from][to] = road;
    if (nodes.find(to) == nodes.end()) {
      nodes[to] = std::unordered_map<intersection_id_t, Road *>();
    }
  }

  void printGraph() {
    for (auto node : this->nodes) {
      std::cout << "node: " << node.first << "\n";
      for (auto neigh : node.second) {
        neigh.second->printRoad();
      }
    }
  }

  ~Graph() {
    for (auto node : this->nodes) {
      for (auto neigh : node.second) {
        delete neigh.second;
      }
    }
  };
};

void parseLine(Graph * graph, std::string line);

// Creates the src, dest car pairs needed for startPlanning
Graph * readGraph(std::string fname);

intersection_id_t findMinNode(std::unordered_map<intersection_id_t, NodeVal> & nodeMap);

void getPath(std::pair<intersection_id_t, intersection_id_t> from_to,
             std::unordered_map<intersection_id_t, NodeVal> & nodeMap,
             std::vector<intersection_id_t> & path);

std::vector<intersection_id_t> dijkstras(
    Graph * graph,
    std::pair<intersection_id_t, intersection_id_t> from_to);

#ifdef START
// This part of code will only perform dijkstras for each car only once and 
// don't care the change of traffic in each road.
intersection_id_t getNextDest(Graph * graph, intersection_id_t current, PerCarInfo * car);

//whenever a car reaches an intersection (including the source, but
//excluding the destination), this function
//will be called to determine what the next intersection should be.
//If you specify an invalid id (not adjacent to the current one), the simulation
//will abort.
//
//Note that you could pre-plan in startPlanning, store the planned route
//in your PerCarInfo, and then just return the next step.
//
//However, you can also do something more interesting to adapt to changing
//traffic conditions
std::vector<intersection_id_t> getNextStep(
    Graph * graph,
    const std::vector<arrival_info_t> & arriving_cars);

#endif

#ifdef ST

intersection_id_t getNextDest(Graph * graph, intersection_id_t current, PerCarInfo * car);

//whenever a car reaches an intersection (including the source, but
//excluding the destination), this function
//will be called to determine what the next intersection should be.
//If you specify an invalid id (not adjacent to the current one), the simulation
//will abort.
//
//Note that you could pre-plan in startPlanning, store the planned route
//in your PerCarInfo, and then just return the next step.
//
//However, you can also do something more interesting to adapt to changing
//traffic conditions
std::vector<intersection_id_t> getNextStep(
    Graph * graph,
    const std::vector<arrival_info_t> & arriving_cars);

#endif

//This will get called once for all cars departing at time T.  You will
//be passed a vector with the from/to intersection ids of each departing car.
//You should then return a vector of the same length with whatever state you want
//associated with each car.
//Note: this is called before the car "arrives" at its source intersection
std::vector<PerCarInfo *> startPlanning(Graph * graph,
                                        const std::vector<start_info_t> & departing_cars);

//when a car reaches its destination, this function will be called.
//you may do any book-keeping you wish (which may be none)
//
//If your PerCarInfo is dynmically allocated, you may safely free it here
//Note: unlike the other two functions (which are called at most ONCE per cycle),
//this is called for each arriving car.
//[This design choice assumes you may wish to do coordinated planning, but
// that resource freeing and other book-keeping does not need coordination]
void carArrived(PerCarInfo * finished_cars);

void cleanup(Graph * g);

#endif
