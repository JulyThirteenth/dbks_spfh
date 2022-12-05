```
    // 使用指南：rs_path、rs_distance
    double minimal_turning_radius = 4.;
    ReedsSheppStateSpace r(minimal_turning_radius);
    ReedsSheppStateSpace::ReedsSheppPath rr;
    double s[3] = {1, 4, 6};
    double g[3] = {2, 3, 6};
    double sample_length = 0.1;
    std::vector<std::vector<double>> rs_path = r.xingshensample(s, g, sample_length);
    double rs_distance = r.distance(s,g);
```
