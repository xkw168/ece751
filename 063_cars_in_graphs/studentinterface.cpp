#include "studentinterface.hpp"

#include "studentapi.hpp"

void parseLine(Graph * graph, std::string line) {
  // id, source, destination, length, speed, maxCars, pairs<cars, speed>
  std::vector<int> items;
  int num;
  std::string strNum;
  size_t i = 0;
  while (true) {
    if (line[i] >= '0' && line[i] <= '9' && i < line.size()) {
      strNum += line[i];
    }
    else {
      if (!strNum.empty()) {
        num = std::stoi(strNum);
        if (num < 0) {
          std::cerr << "Error: unexpected negative value in file.\n";
          exit(EXIT_FAILURE);
        }
        items.push_back(num);
        strNum.clear();
      }
    }
    if (i == line.size()) {
      break;
    }
    i++;
  }
  // check if it's valid
  if ((items.size() < 6) || (items.size() % 2 != 0)) {
    std::cerr << "Error: illegal format in file(wrong nums of number in some line)\n";
    exit(EXIT_FAILURE);
  }
  graph->addRoad(new Road(items));
}

// Creates the src, dest car pairs needed for startPlanning
Graph * readGraph(std::string fname) {
  Graph * graph = new Graph();
  std::string line;
  std::ifstream ifs(fname);
  if (ifs.fail()) {
    std::cerr << "file not exist!!!\n";
    exit(EXIT_FAILURE);
  }
  while (std::getline(ifs, line)) {
    parseLine(graph, line);
  }
  ifs.close();
  return graph;
}

intersection_id_t findMinNode(std::unordered_map<intersection_id_t, NodeVal> & nodeMap) {
  // find the intersection that has the minimun distance
  size_t minDist = UINT_MAX;
  size_t minNode;
  for (auto n : nodeMap) {
    if (!n.second.isMarked && (n.second.distance < minDist)) {
      minNode = n.first;
      minDist = n.second.distance;
    }
  }
  return minNode;
}

void getPath(std::pair<intersection_id_t, intersection_id_t> from_to,
             std::unordered_map<intersection_id_t, NodeVal> & nodeMap,
             std::vector<intersection_id_t> & path) {
  // Called at the end of dijkstras, backtrack the path
  path.clear();
  intersection_id_t tmp = from_to.second;
  while (tmp != from_to.first) {
    path.push_back(tmp);
    tmp = nodeMap[tmp].prev;
  }
  path.push_back(tmp);
  std::reverse(path.begin(), path.end());
}

std::vector<intersection_id_t> dijkstras(
    Graph * graph,
    std::pair<intersection_id_t, intersection_id_t> from_to) {
  // perform dijkstras algorithm
  std::vector<intersection_id_t> path;
  std::unordered_map<intersection_id_t, std::unordered_map<intersection_id_t, Road *> >
      allNodeVals = graph->getNodes();
  std::unordered_map<intersection_id_t, NodeVal> nodeMap;

  // initialize all nodes(dist --- UINT_MAX, prev --- -1)
  for (auto & n : allNodeVals) {
    // set the distance of starting node to 0
    NodeVal val;
    val.distance = (n.first == from_to.first ? 0 : UINT_MAX);
    val.isMarked = false;
    nodeMap.emplace(n.first, val);
  }

  size_t cnt = 0;
  while (cnt < nodeMap.size()) {
    intersection_id_t minNode = findMinNode(nodeMap);
    nodeMap[minNode].isMarked = true;
    // update all neighbours
    for (auto neigh : allNodeVals[minNode]) {
      intersection_id_t neightId = neigh.first;
      Road * road = neigh.second;
      // only update the distance when there is a better offer
      if (nodeMap[neightId].distance > nodeMap[minNode].distance + road->getTime()) {
        nodeMap[neightId].distance = nodeMap[minNode].distance + road->getTime();
        nodeMap[neightId].prev = minNode;
      }
    }
    cnt++;
  }
  getPath(from_to, nodeMap, path);

  return path;
}

#ifdef START
// This part of code will only perform dijkstras for each car only once and don't care the change of traffic in each road.
intersection_id_t getNextDest(Graph * graph,
                              intersection_id_t current,
                              PerCarInfo * car) {
  // simply retrieve the next step from PerCarInfo
  std::vector<intersection_id_t> path = car->getPath();
  for (size_t i = 0; i < path.size(); i++) {
    if (path[i] == current) {
      return path[i + 1];
    }
  }
  return -1;
}

std::vector<intersection_id_t> getNextStep(
    Graph * graph,
    const std::vector<arrival_info_t> & arriving_cars) {
  // arrival_info_t --- std::pair<intersection_id_t, PerCarInfo *>
  std::vector<intersection_id_t> nextDest;
  for (size_t i = 0; i < arriving_cars.size(); i++) {
    nextDest.push_back(
        getNextDest(graph, arriving_cars[i].first, arriving_cars[i].second));
  }
  return nextDest;
}

#endif

#ifdef ST

intersection_id_t getNextDest(Graph * graph,
                              intersection_id_t current,
                              PerCarInfo * car) {
  intersection_id_t to = car->getDest();
  // recalculate the path
  std::vector<intersection_id_t> path = dijkstras(graph, std::make_pair(current, to));
  return path[1];
}

std::vector<intersection_id_t> getNextStep(
    Graph * graph,
    const std::vector<arrival_info_t> & arriving_cars) {
  // arrival_info_t --- std::pair<intersection_id_t, PerCarInfo *>
  std::vector<intersection_id_t> nextDest;
  for (size_t i = 0; i < arriving_cars.size(); i++) {
    nextDest.push_back(
        getNextDest(graph, arriving_cars[i].first, arriving_cars[i].second));
  }
  return nextDest;
}

#endif

//This will get called once for all cars departing at time T.  You will
//be passed a vector with the from/to intersection ids of each departing car.
//You should then return a vector of the same length with whatever state you want
//associated with each car.
//Note: this is called before the car "arrives" at its source intersection
std::vector<PerCarInfo *> startPlanning(
    Graph * graph,
    const std::vector<start_info_t> & departing_cars) {
  // from_to_pair_t --- std::pair<intersection_id_t, intersection_id_t>
  // start_info_t --- std::pair<car_id_t, from_to_pair_t>
  std::vector<PerCarInfo *> carsInfo;
  for (auto car : departing_cars) {
    carsInfo.push_back(new PerCarInfo(car.first, dijkstras(graph, car.second)));
  }
  return carsInfo;
}

//when a car reaches its destination, this function will be called.
//you may do any book-keeping you wish (which may be none)
//
//If your PerCarInfo is dynmically allocated, you may safely free it here
//Note: unlike the other two functions (which are called at most ONCE per cycle),
//this is called for each arriving car.
//[This design choice assumes you may wish to do coordinated planning, but
// that resource freeing and other book-keeping does not need coordination]
void carArrived(PerCarInfo * finished_cars) {
  delete finished_cars;
}

void cleanup(Graph * g) {
  delete g;
}
