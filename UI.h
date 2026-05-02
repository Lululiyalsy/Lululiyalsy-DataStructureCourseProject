#pragma once
#include "SimulationManager.h"
#include <string>

#define NOMINMAX
#include <graphics.h>

const int WINDOW_WIDTH = 1500;
const int WINDOW_HEIGHT = 1500;

enum class AppState {
    Welcome,
    Simulation
};

struct Button {
    int x, y, width, height;
    std::wstring text;

    void draw() const;
    bool isHovered(int mx, int my) const;
};

class SubwaySimulationUI {
public:
    SubwaySimulationUI(SubwayGraph& g, SimulationManager& sm);
    ~SubwaySimulationUI();
    void run();

private:
    void processEvents();
    void render();
    void floorUp();
    void floorDown();
    void zoomIn(int mx, int my);
    void zoomOut(int mx, int my);
    void drawMap();
    void drawPassengers();

    SubwayGraph& graph;
    SimulationManager& simManager;

    AppState currentState;
    int currentFloor;
    float viewScale;
    float offsetX;
    float offsetY;
    bool isDragging;
    int dragStartX;
    int dragStartY;
    bool isRunning;
    int simulationStep;
    int maxSimulationSteps;

    Button startBtn;
    Button upBtn;
    Button downBtn;
};
