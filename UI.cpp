#include "UI.h"
#include "SubwayGraph.h"
#include "Node.h"
#include "Passenger.h"
#include <windows.h>
#include <cmath>

void Button::draw() const {
    setfillcolor(LIGHTGRAY);
    setlinecolor(BLACK);
    fillrectangle(x, y, x + width, y + height);
    rectangle(x, y, x + width, y + height);

    settextcolor(BLACK);
    LOGFONT f = {};
    f.lfHeight = 20;
    f.lfWeight = FW_BOLD;
    wcscpy_s(f.lfFaceName, L"微软雅黑");
    settextstyle(&f);

    int tw = textwidth(text.c_str());
    int th = textheight(text.c_str());
    outtextxy(x + (width - tw) / 2, y + (height - th) / 2, text.c_str());
}

bool Button::isHovered(int mx, int my) const {
    return mx >= x && mx <= x + width && my >= y && my <= y + height;
}

SubwaySimulationUI::SubwaySimulationUI(SubwayGraph& g, SimulationManager& sm)
    : graph(g), simManager(sm),
    currentState(AppState::Welcome),
    currentFloor(1),
    viewScale(1.0f),
    offsetX(0.0f),
    offsetY(0.0f),
    isDragging(false),
    dragStartX(0),
    dragStartY(0),
    isRunning(true),
    simulationStep(0),
    maxSimulationSteps(14400) {
    initgraph(WINDOW_WIDTH, WINDOW_HEIGHT, EX_SHOWCONSOLE);
    BeginBatchDraw();

    startBtn = { (WINDOW_WIDTH - 200) / 2, WINDOW_HEIGHT / 2 + 50, 200, 50, L"开始仿真" };

    int btnX = WINDOW_WIDTH - 130;
    upBtn = { btnX, 150, 110, 40, L"▲ 上层" };
    downBtn = { btnX, 200, 110, 40, L"▼ 下层" };
}

SubwaySimulationUI::~SubwaySimulationUI() {
    EndBatchDraw();
    closegraph();
}

void SubwaySimulationUI::run() {
    while (isRunning) {
        processEvents();
        render();
        Sleep(16);
    }
    // 仿真结束时输出最终报告
    simManager.exportFinalReport();
}

void SubwaySimulationUI::processEvents() {
    ExMessage msg;
    while (peekmessage(&msg, EM_MOUSE | EM_KEY)) {
        if (msg.message == WM_KEYDOWN) {
            if (msg.vkcode == VK_ESCAPE || msg.vkcode == 'Q' || msg.vkcode == 'q') {
                isRunning = false;
            }
        }

        if (msg.message == WM_LBUTTONDOWN) {
            if (currentState == AppState::Welcome) {
                if (startBtn.isHovered(msg.x, msg.y)) {
                    currentState = AppState::Simulation;
                }
            } else if (currentState == AppState::Simulation) {
                if (upBtn.isHovered(msg.x, msg.y)) floorUp();
                if (downBtn.isHovered(msg.x, msg.y)) floorDown();
            }
        }
    }

    if (msg.message == WM_RBUTTONDOWN) {
        if (currentState == AppState::Simulation) {
            isDragging = true;
            dragStartX = msg.x;
            dragStartY = msg.y;
        }
    } else if (msg.message == WM_RBUTTONUP) {
        isDragging = false;
    } else if (msg.message == WM_MOUSEMOVE) {
        if (isDragging && currentState == AppState::Simulation) {
            offsetX += (msg.x - dragStartX);
            offsetY += (msg.y - dragStartY);
            dragStartX = msg.x;
            dragStartY = msg.y;
        }
    }

    if (msg.message == WM_MOUSEWHEEL) {
        if (currentState == AppState::Simulation) {
            if (msg.wheel > 0) zoomIn(msg.x, msg.y);
            else if (msg.wheel < 0) zoomOut(msg.x, msg.y);
        }
    }
}

void SubwaySimulationUI::render() {
    setbkcolor(WHITE);
    cleardevice();

    if (currentState == AppState::Welcome) {
        settextcolor(BLACK);
        LOGFONT titleFont = {};
        titleFont.lfHeight = 50;
        titleFont.lfWeight = FW_HEAVY;
        wcscpy_s(titleFont.lfFaceName, L"微软雅黑");
        settextstyle(&titleFont);

        std::wstring titleText = L"地铁站人群流动仿真系统";
        int tw = textwidth(titleText.c_str());
        outtextxy((WINDOW_WIDTH - tw) / 2, WINDOW_HEIGHT / 2 - 100, titleText.c_str());

        startBtn.draw();
    } else if (currentState == AppState::Simulation) {
        simManager.stepOnce();
        simulationStep++;

        if (simulationStep >= maxSimulationSteps) {
            isRunning = false;
        }

        drawMap();
        drawPassengers();

        settextcolor(BLACK);
        LOGFONT mainFont = {};
        mainFont.lfHeight = 28;
        mainFont.lfWeight = FW_BOLD;
        wcscpy_s(mainFont.lfFaceName, L"微软雅黑");
        settextstyle(&mainFont);

        outtextxy(20, 20, L"【地铁站流动实时监控面板】");

        std::wstring floorText = L"当前楼层: " + std::to_wstring(currentFloor) + L" 层";
        outtextxy(20, 60, floorText.c_str());

        wchar_t scaleBuffer[32];
        swprintf(scaleBuffer, 32, L"视图倍率: %.1fx", viewScale);
        outtextxy(20, 100, scaleBuffer);

        upBtn.draw();
        downBtn.draw();
    }

    FlushBatchDraw();
}

void SubwaySimulationUI::floorUp() {
    if (currentFloor == -1) currentFloor = 1;
    else if (currentFloor < 7) currentFloor++;
}

void SubwaySimulationUI::floorDown() {
    if (currentFloor == 1) currentFloor = -1;
    else if (currentFloor > -3) currentFloor--;
}

void SubwaySimulationUI::zoomIn(int mx, int my) {
    float oldScale = viewScale;
    float newScale = viewScale + 0.1f;
    viewScale = (newScale < 3.0f) ? newScale : 3.0f;

    if (viewScale != oldScale) {
        float logicalX = (mx - offsetX) / oldScale;
        float logicalY = (my - offsetY) / oldScale;
        offsetX = mx - logicalX * viewScale;
        offsetY = my - logicalY * viewScale;
    }
}

void SubwaySimulationUI::zoomOut(int mx, int my) {
    float oldScale = viewScale;
    float newScale = viewScale - 0.1f;
    viewScale = (newScale > 0.5f) ? newScale : 0.5f;

    if (viewScale != oldScale) {
        float logicalX = (mx - offsetX) / oldScale;
        float logicalY = (my - offsetY) / oldScale;
        offsetX = mx - logicalX * viewScale;
        offsetY = my - logicalY * viewScale;
    }
}

void SubwaySimulationUI::drawMap() {
    setlinecolor(LIGHTGRAY);
    for (size_t i = 0; i < graph.getAllNodes().size(); ++i) {
        const auto& fromNode = graph.getAllNodes()[i];
        if (fromNode->getFloor() == currentFloor) {
            const auto& edges = graph.getNeighbors(i);
            for (const auto& edge : edges) {
                int toIdx = edge.getToIndex();
                const auto& toNode = graph.getNode(toIdx);
                if (toNode && toNode->getFloor() == currentFloor) {
                    MYPOINT fromPos = fromNode->getPos();
                    MYPOINT toPos = toNode->getPos();
                    float rx1 = fromPos.x * viewScale + offsetX;
                    float ry1 = fromPos.y * viewScale + offsetY;
                    float rx2 = toPos.x * viewScale + offsetX;
                    float ry2 = toPos.y * viewScale + offsetY;

                    float dx = rx2 - rx1;
                    float dy = ry2 - ry1;
                    float length = sqrt(dx * dx + dy * dy);

                    if (length > 0) {
                        float ux = dx / length;
                        float uy = dy / length;
                        float vx = -uy;
                        float vy = ux;
                        float offset = 5.0f * viewScale;

                        line(static_cast<int>(rx1 + vx * offset), static_cast<int>(ry1 + vy * offset),
                            static_cast<int>(rx2 + vx * offset), static_cast<int>(ry2 + vy * offset));
                        line(static_cast<int>(rx1 - vx * offset), static_cast<int>(ry1 - vy * offset),
                            static_cast<int>(rx2 - vx * offset), static_cast<int>(ry2 - vy * offset));
                    }
                }
            }
        }
    }

    setbkmode(TRANSPARENT);
    settextcolor(BLACK);
    LOGFONT f = {};
    f.lfHeight = static_cast<long>(20 * viewScale);
    f.lfWeight = FW_NORMAL;
    wcscpy_s(f.lfFaceName, L"微软雅黑");
    settextstyle(&f);

    for (const auto& node : graph.getAllNodes()) {
        if (node->getFloor() == currentFloor) {
            MYPOINT pos = node->getPos();
            float rx = pos.x * viewScale + offsetX;
            float ry = pos.y * viewScale + offsetY;

            setlinecolor(BLACK);
            setfillcolor(WHITE);

            std::string typeCode = node->getTypeCode();
            if (typeCode == "HALL") {
                float rw = 120 * viewScale;
                float rh = 40 * viewScale;
                fillrectangle(static_cast<int>(rx - rw / 2), static_cast<int>(ry - rh / 2),
                    static_cast<int>(rx + rw / 2), static_cast<int>(ry + rh / 2));
                rectangle(static_cast<int>(rx - rw / 2), static_cast<int>(ry - rh / 2),
                    static_cast<int>(rx + rw / 2), static_cast<int>(ry + rh / 2));
            } else if (typeCode == "PLATFORM") {
                float rw = 40 * viewScale;
                float rh = 150 * viewScale;
                fillrectangle(static_cast<int>(rx - rw / 2), static_cast<int>(ry - rh / 2),
                    static_cast<int>(rx + rw / 2), static_cast<int>(ry + rh / 2));
                rectangle(static_cast<int>(rx - rw / 2), static_cast<int>(ry - rh / 2),
                    static_cast<int>(rx + rw / 2), static_cast<int>(ry + rh / 2));
            } else if (typeCode == "TICKET") {
                float rw = 40 * viewScale;
                float rh = 25 * viewScale;
                fillroundrect(static_cast<int>(rx - rw / 2), static_cast<int>(ry - rh / 2),
                    static_cast<int>(rx + rw / 2), static_cast<int>(ry + rh / 2), 10, 10);
                roundrect(static_cast<int>(rx - rw / 2), static_cast<int>(ry - rh / 2),
                    static_cast<int>(rx + rw / 2), static_cast<int>(ry + rh / 2), 10, 10);
            } else if (typeCode == "GATE") {
                int rxRadius = static_cast<int>(25 * viewScale);
                int ryRadius = static_cast<int>(12 * viewScale);
                fillellipse(static_cast<int>(rx - rxRadius), static_cast<int>(ry - ryRadius),
                    static_cast<int>(rx + rxRadius), static_cast<int>(ry + ryRadius));
                ellipse(static_cast<int>(rx - rxRadius), static_cast<int>(ry - ryRadius),
                    static_cast<int>(rx + rxRadius), static_cast<int>(ry + ryRadius));
            } else if (typeCode == "SECURITY") {
                POINT pts[4];
                int rW = static_cast<int>(30 * viewScale);
                int rH = static_cast<int>(20 * viewScale);
                pts[0].x = static_cast<int>(rx); pts[0].y = static_cast<int>(ry - rH);
                pts[1].x = static_cast<int>(rx + rW); pts[1].y = static_cast<int>(ry);
                pts[2].x = static_cast<int>(rx); pts[2].y = static_cast<int>(ry + rH);
                pts[3].x = static_cast<int>(rx - rW); pts[3].y = static_cast<int>(ry);
                fillpolygon(pts, 4);
                polygon(pts, 4);
            } else if (typeCode == "EXIT") {
                int radius = static_cast<int>(20 * viewScale);
                fillcircle(static_cast<int>(rx), static_cast<int>(ry), radius);
                circle(static_cast<int>(rx), static_cast<int>(ry), radius);
            } else if (typeCode == "STAIR") {
                POINT pts[3];
                int rW = static_cast<int>(30 * viewScale);
                int rH = static_cast<int>(30 * viewScale);
                pts[0].x = static_cast<int>(rx); pts[0].y = static_cast<int>(ry - rH / 2);
                pts[1].x = static_cast<int>(rx + rW); pts[1].y = static_cast<int>(ry + rH / 2);
                pts[2].x = static_cast<int>(rx - rW); pts[2].y = static_cast<int>(ry + rH / 2);
                fillpolygon(pts, 3);
                polygon(pts, 3);
            }

            std::wstring label(typeCode.begin(), typeCode.end());
            int tw = textwidth(label.c_str());
            int th = textheight(label.c_str());
            outtextxy(static_cast<int>(rx) - tw / 2, static_cast<int>(ry) - th / 2, label.c_str());
        }
    }
}

void SubwaySimulationUI::drawPassengers() {
    setfillcolor(RED);
    setlinecolor(RED);

    const float baseRadius = 5.0f;

    const auto& passengers = simManager.getPassengers();
    for (const auto& passenger : passengers) {
        if (passenger.getFloor() == currentFloor) {
            if (passenger.getState() == PassengerState::IN_TRANSIT) {
                int fromNodeId = passenger.getCurrentEdgeFrom();
                int toNodeId = passenger.getCurrentEdgeTo();
                double transitProgress = passenger.getTransitProgress();

                if (fromNodeId != -1 && toNodeId != -1) {
                    const auto& fromNode = graph.getNode(fromNodeId);
                    const auto& toNode = graph.getNode(toNodeId);

                    if (fromNode && toNode) {
                        MYPOINT fromPos = fromNode->getPos();
                        MYPOINT toPos = toNode->getPos();

                        double clampedProgress = std::max(0.0, std::min(1.0, transitProgress));
                        float x = fromPos.x + (toPos.x - fromPos.x) * clampedProgress;
                        float y = fromPos.y + (toPos.y - fromPos.y) * clampedProgress;

                        float renderX = x * viewScale + offsetX;
                        float renderY = y * viewScale + offsetY;
                        float renderRadius = baseRadius * viewScale;
                        solidcircle(static_cast<int>(renderX), static_cast<int>(renderY), static_cast<int>(renderRadius));
                    }
                }
            } else {
                MYPOINT pos = passenger.getPosition();
                float renderX = pos.x * viewScale + offsetX;
                float renderY = pos.y * viewScale + offsetY;
                float renderRadius = baseRadius * viewScale;
                solidcircle(static_cast<int>(renderX), static_cast<int>(renderY), static_cast<int>(renderRadius));
            }
        }
    }
}