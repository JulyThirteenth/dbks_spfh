```
int dubinsTransform(double q[3], double x, void *user_data)
{
    std::vector<double> user_data_node;
    user_data_node.push_back(q[0]);
    user_data_node.push_back(q[1]);
    user_data_node.push_back(q[2]);
    std::vector<std::vector<double>> * dubins_path = 
        (std::vector<std::vector<double>> *)user_data;
    dubins_path->push_back(user_data_node);
    return 0;
}
```
```
//使用指南：dubins_path、dubins_distance
DubinsPath temp;
double minimal_turning_radius = 4.;
dubins_shortest_path(&temp, m_start, m_end, minimal_turning_radius);
std::vector<std::vector<double>> dubins_path;
double sample_length = 0.1;
dubins_path_sample_many(&temp, sample_length, dubinsTransform, &dubins_path);
double dubins_distance = dubins_path_length(&temp);
```