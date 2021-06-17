#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "load.h"
#include "geometry.h"

class Visualization
{
private:
    std::string window_name;
    double scale;

public:
    cv::Mat img;
    DrawConfig config;
    /* Initialize the window 
     * @Param:
     *      data: the loaded copper data
     *      name: the name of output window
     *      scale: the scale to draw (pixel/mm)
     */
    Visualization(const Load &data, std::string _name, double _scale = 100);

    /* Draw each component and show */
    void show(const Load &data);

    void draw();
};

#endif