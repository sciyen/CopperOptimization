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
    cv::Mat img;
    std::string window_name;
    double scale;
    DrawConfig config;

public:
    /* Initialize the window 
     * @Param:
     *      data: the loaded copper data
     *      name: the name of output window
     *      scale: the scale to draw (pixel/mm)
     */
    Visualization(const Load &data, std::string _name, double _scale = 100);

    /* Draw each component and show */
    void show(const Load &data);
};

#endif