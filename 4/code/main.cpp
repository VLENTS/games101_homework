#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 6) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t, int l, int r) 
{
    // TODO: Implement de Casteljau's algorithm
    cv::Point2f p_1 = control_points[l], p_2 = control_points[r];
    if(r - l > 1) {
        p_1 = recursive_bezier(control_points, t, l, r - 1);
        p_2 = recursive_bezier(control_points, t, l + 1, r);
    }
    
    return p_1 * (1 - t) + p_2 * t;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t, 0, control_points.size() - 1); 

        // 选择靠近曲线点一侧的四个像素块，根据曲线点与像素中心的距离分配颜色
        int x = floor(point.x), y = floor(point.y);
        int dx = point.x < x + 0.5 ? x - 1 : x, dy = point.y < y + 0.5 ? y - 1 : y;
        float d = sqrt(pow(x + 0.5 - point.x, 2.0) + pow(y + 0.5 - point.y, 2.0));
        
        for(int i = 0; i < 2; i++)
            for(int j = 0; j < 2; j++) {
                int nx = dx + i, ny = dy + j;
                if(nx == x && ny == y)
                    window.at<cv::Vec3b>(y, x)[1] = 255;
                else {
                    float distance = sqrt(pow(nx + 0.5 - point.x, 2.0) + pow(ny + 0.5 - point.y, 2.0));
                    window.at<cv::Vec3b>(ny, nx)[1] = 
                        std::max((int)(window.at<cv::Vec3b>(ny, nx)[1]), (int)(255 * d / distance));
                }
            }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 6) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
