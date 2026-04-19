#include "SimulationManager.h"
#include "SubwayGraph.h"
#include <iostream>

int main() {
    std::cout << "\u5730\u94c1\u7ad9\u4eba\u7fa4\u4eff\u771f\u7cfb\u7edf\u542f\u52a8..." << std::endl;

    SubwayGraph graph;

    if (!graph.loadFromCSV("subway_config.csv")) {
        std::cerr << "\u9519\u8bef\uff1a\u65e0\u6cd5\u52a0\u8f7d\u914d\u7f6e\u6587\u4ef6\uff0c\u8bf7\u68c0\u67e5\u6587\u4ef6\u662f\u5426\u5728 Debug \u76ee\u5f55\u4e0b\u3002" << std::endl;
        return -1;
    }

    std::cout << "\u6210\u529f\u52a0\u8f7d " << graph.getAllNodes().size() << " \u4e2a\u8282\u70b9\u3002" << std::endl;

    SimulationManager sim(graph);
    sim.run(14400);

    return 0;
}