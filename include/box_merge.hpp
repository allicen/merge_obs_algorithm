#ifndef BOX_MERGE_HPP
#define BOX_MERGE_HPP


class BoxMerge {
  public:
    BoxMerge(double length_, double width_, double height_, double x_, double y_, double z_):
        length(length_), width(width_), height(height_), x(x_), y(y_), z(z_) {
    }

    double length; // by Y
    double width; // by X
    double height; // by Z
    double x;
    double y;
    double z;
};

#endif