#include "visualization.h"

Visualization::Visualization(const Load &data, std::string _name, double _scale)
{
    double width = data.bbox.xmax() - data.bbox.xmin();
    double height = data.bbox.ymax() - data.bbox.ymin();
    std::cout << "size: " << width << ',' << height << std::endl;
    window_name = _name;
    scale = _scale;
    config.s_xmin = data.bbox.xmin();
    config.s_ymin = data.bbox.ymin();
    config.s_w = width;
    config.s_h = height;
    config.t_w = width * scale;
    config.t_h = height * scale;
    config.scale = scale;
    img = cv::Mat::zeros(cv::Size(config.t_w, config.t_h), CV_8UC3);
    std::cout << "size: " << config.t_w << ',' << config.t_h << std::endl;
}

void Visualization::reset()
{
    img = cv::Mat::zeros(cv::Size(config.t_w, config.t_h), CV_8UC3);
}

void Visualization::show(const Load &data)
{
    for (auto node : data.nodes) {
        std::cout << "drawing " << node.node_type << std::endl;
        node.draw(img, this->config);
    }
}

void Visualization::draw()
{
    cv::imshow(window_name, img);
    cv::waitKey(0);
}

void Visualization::imwrite(std::string filename)
{
    cv::imwrite(filename, img);
}