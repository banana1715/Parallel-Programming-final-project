/**
 *  This is the main file that calls the RRT* algorithm. 
 *  First, the algorithm generates a plan (vector of points) which is a first viable solution.
 *  Next, it calls the RRT* algorithm on the previouslly built plan to optimize it.
 */

#include"RRTstar.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "BMP.h"
#include <chrono>
//main function
int main(int argc, char **argv)
{
    RRTSTAR* rrtstar;
    int width = 500, height = 500;
    std::string output_path = "";
    std::chrono::duration<double> total_planner_time(0);
#ifdef TEST_TIME
    int ITER = 10;
    std::chrono::duration<double> total_findNearest_time(0);
    std::chrono::duration<double> total_findNearNeighbors_time(0);
    std::chrono::duration<double> total_findParent_time(0);
    for (int iter = 0; iter < ITER; iter++) {
#endif
    
    if (argc == 1) // use default setting
    {
        //define start and end positions
        Point start_pos(10,490);
        Point end_pos(490, 10);
        //define the raduis for RRT* algorithm (Within a radius of r, RRT* will find all neighbour nodes of a new node).
        float rrt_radius = 25;
        //define the radius to check if the last node in the tree is close to the end position
        float end_thresh = 10;
        //instantiate RRTSTAR class
        rrtstar = new RRTSTAR(start_pos,end_pos, rrt_radius, end_thresh);

        //set the width and height of the world
        rrtstar->world->setWorldWidth(width);
        rrtstar->world->setWorldHeight(height);

        // set step size and max iterations. If the values are not set, the default values are max_iter=5000 and step_size=10.0
        rrtstar->setMaxIterations(10000);
        rrtstar->setStepSize(10.0);

        //Create obstacles
        // //Obstacle 1
        // Point ob1_1(0, 400); //position of the top left point of obstacle 1
        // Point ob1_2(350, 350.0); //position of the bottom right point of obstacle 1
        // rrtstar->world->addObstacle(ob1_1, ob1_2);//create obstacle 1
        // //Obstacle 2;
        // Point ob2_1(150, 300.0); //position of the top left point of obstacle 2
        // Point ob2_2(500, 250.0); //position of the bottom right point of obstacle 2
        // rrtstar->world->addObstacle(ob2_1, ob2_2);//create obstacle 2
        // //Obstacle 3;
        // Point ob3_1(  0, 200); //position of the top left point of obstacle 3
        // Point ob3_2(350, 150); //position of the bottom right point of obstacle 3
        // rrtstar->world->addObstacle(ob3_1, ob3_2);//create obstacle 3
        // //Obstacle 3;
        // Point ob4_1(150, 125); //position of the top left point of obstacle 3
        // Point ob4_2(500,  75); //position of the bottom right point of obstacle 3
        // rrtstar->world->addObstacle(ob4_1, ob4_2);//create obstacle 3
        for (int j = 0; j < 6; j++)
        for (int i = 5-j; i >= 0; i--)
        {
            Point ob1(i*75 + 50, j*75 + 50);
            Point ob2(i*75 + 100, j*75 + 100);
            rrtstar->world->addObstacle(ob1, ob2);
        }
        //Save obstacles to  file;
        rrtstar->world->saveObsToFile("Mfiles//Obstacles.txt");
    } else if (argc == 2 || argc == 3) {
        if (argc == 3)
            output_path = argv[2];
        float x1, y1, x2, y2;
        std::ifstream ifs(argv[1]);

        if (!ifs.is_open()) {
            std::cerr << "Failed to open file.\n";
            exit(-1);
        }
        ifs >> width >> height; // input height and width
        ifs >> x1 >> y1 >> x2 >> y2; // input start and end points

        //define start and end positions
        Point start_pos(x1,y1);
        Point end_pos(x2, y2);

        //define the raduis for RRT* algorithm (Within a radius of r, RRT* will find all neighbour nodes of a new node).
        float rrt_radius;
        //define the radius to check if the last node in the tree is close to the end position
        float end_thresh;

        ifs >> rrt_radius >> end_thresh;
        //instantiate RRTSTAR class
        rrtstar = new RRTSTAR(start_pos,end_pos, rrt_radius, end_thresh);

        //set the width and height of the world
        rrtstar->world->setWorldWidth(width);
        rrtstar->world->setWorldHeight(height);

        int MAX_ITER, StepSize;
        ifs >> MAX_ITER >> StepSize;
        // set step size and max iterations. If the values are not set, the default values are max_iter=5000 and step_size=10.0
        rrtstar->setMaxIterations(MAX_ITER);
        rrtstar->setStepSize(StepSize);

        while (ifs >> x1 >> y1 >> x2 >> y2) {
            Point ob1(std::min(x1, x2), std::max(y1, y2));
            Point ob2(std::max(x1, x2), std::min(y1, y2));
            rrtstar->world->addObstacle(ob1, ob2);
        }
        //Save obstacles to  file;
        rrtstar->world->saveObsToFile("Mfiles//Obstacles.txt");
    } else {
        fprintf(stderr, "Usage: %s {Map file}\n", argv[0]);
        exit(-1);
    }
    

    // sert up a Graph class
    Graph g(width, height, output_path);

    // Initialize the graph background
    g.SetUpBackGround();
    rrtstar->world->SetAllObs(g);

    //clear saved paths from previous run
    rrtstar->savePlanToFile({}, "Mfiles//first_viable_path.txt", {});
    rrtstar->savePlanToFile({}, "Mfiles//Path_after_MAX_ITER.txt", {});


    // RRT* Algorithm
    /*
     Description of RRT* algorithm: 
    1. Pick a random node "N_rand".
    2. Find the closest node "N_Nearest" from explored nodes to branch out towards "N_rand".
    3. Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away. The interpolated Node is "N_new"
    4.  Check if an obstacle is between new node and nearest nod.
    5. Update cost of reaching "N_new" from "N_Nearest", treat it as "cmin". For now, "N_Nearest" acts as the parent node of "N_new".
    6. Find nearest neighbors with a given radius from "N_new", call them "N_near"
    7. In all members of "N_near", check if "N_new" can be reached from a different parent node with cost lower than Cmin, and without colliding
    with the obstacle. Select the node that results in the least cost and update the parent of "N_new".
    8. Add N_new to node list.
    9. Rewire the tree if possible. Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. If so, rewire the tree and
    add them as the children of "N_new" and update the cost of the path.
    10. Continue until maximum number of nodes is reached or goal is hit.
    */

    std::cout << "Starting RRT* Algorithm..." << std::endl;
    //search for the first viable solution
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Point> initial_solution =rrtstar->planner(g);
    auto end_time = std::chrono::high_resolution_clock::now(); // 記錄 planner 結束時間
    total_planner_time += (end_time - start_time);
    //save initial solution
    rrtstar->savePlanToFile(initial_solution, "Mfiles//first_viable_path.txt", "First viable solution . This file contains vector of points of the generated path.");
    if (!initial_solution.empty()) {
        std::cout << "First Viable Solution Obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->lastnode->cost << std::endl;
        std::cout << "Number of Node: " << rrtstar->GetNumNodes() << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }

    int X, Y, old_X = -1, old_Y = -1;
    for (size_t i = 0; i < initial_solution.size(); i++) {
        X = round(initial_solution[i].m_x);
        Y = round(initial_solution[i].m_y);
        int pixel = 3;
        g.DrawRect(std::max(X - pixel, 0), std::max(Y - pixel, 0), std::min(X + pixel, width - 1), std::min(Y + pixel, height - 1), {255, 0, 64});
        if (old_X >= 0 && old_Y >= 0)
            g.DrawLine(X, Y, old_X, old_Y, {255, 0, 64});
        old_X = X;
        old_Y = Y;
    }
    g.WriteImage();

    g.CheckPoint();


    std::vector<Point> optimized_solution;
    //search for the optimized paths
    
    while (rrtstar->getCurrentIterations() < rrtstar->getMaxIterations() && !initial_solution.empty())
    {
        std::cout << "=========================================================================" << std::endl;
        std::cout << "The algorithm continues iterating on the current plan to improve the plan" << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now(); // 記錄 planner 開始時間
        optimized_solution = rrtstar->planner(g);
        auto end_time = std::chrono::high_resolution_clock::now(); // 記錄 planner 結束時間
        total_planner_time += (end_time - start_time);
        std::cout << "More optimal solution has obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->m_cost_bestpath << std::endl;
        std::cout << "Number of Node: " << rrtstar->GetNumNodes() << std::endl;
#ifdef UPDATE_ONCE
    }
#endif
        g.Restore();
        old_X = old_Y = -1;

        for (size_t i = 0; i < optimized_solution.size(); i++) {
            X = round(optimized_solution[i].m_x);
            Y = round(optimized_solution[i].m_y);
            int pixel = 3;
            g.DrawStar(X, Y, pixel, {64, 0, 255});
            if (old_X >= 0 && old_Y >= 0)
                g.DrawLine(X, Y, old_X, old_Y, {64, 0, 255});
            old_X = X;
            old_Y = Y;
        }
    
        g.WriteImage();
#ifndef UPDATE_ONCE
    }
#endif
    //save optimized solution
    std::cout << "Total time spent in planner : " << total_planner_time.count() << " seconds" << std::endl;
    std::cout << "[findNearest()]: " << rrtstar->findNearest_time.count() << " seconds (" << (rrtstar->findNearest_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    std::cout << "[findNearNeighbors_time()]: " << rrtstar->findNearNeighbors_time.count() << " seconds (" << (rrtstar->findNearNeighbors_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    std::cout << "[findParent_time()]: " << rrtstar->findParent_time.count() << " seconds (" << (rrtstar->findParent_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    rrtstar->savePlanToFile(optimized_solution, "Mfiles//Path_after_MAX_ITER.txt", " Optimized Solution after maximum provided iteration.This file contains vector of points of the generated path.");
    if (!optimized_solution.empty()) {
        std::cout << "Exceeded max iterations!" << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }
#ifdef TEST_TIME
    total_findNearest_time += rrtstar->findNearest_time;
    total_findNearNeighbors_time += rrtstar->findNearNeighbors_time;
    total_findParent_time += rrtstar->findParent_time;
    }
    //free up the memory

    std::cout << "Total time spent in planner : " << (total_planner_time.count() / ITER) << " seconds" << std::endl;
    std::cout << "[findNearest()]             : " << (total_findNearest_time.count() / ITER) << " seconds (" << (total_findNearest_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    std::cout << "[findNearNeighbors_time()]  : " << (total_findNearNeighbors_time.count() / ITER) << " seconds (" << (total_findNearNeighbors_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
    std::cout << "[findParent_time()]         : " << (total_findParent_time.count() / ITER) << " seconds (" << (total_findParent_time.count() / total_planner_time.count() * 100) << "%)" << std::endl;
#endif
    // WriteBMP("output.bmp", width, height, "Mfiles/Obstacles.txt", "Mfiles/Path_after_MAX_ITER.txt", "Mfiles/first_viable_path.txt");
    delete rrtstar;
}