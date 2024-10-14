#pragma once
#ifndef COLOR_HPP
#define COLOR_HPP

#include <stdlib.h>
#include <stdio.h>


class ColorInfo {
    public:
        ColorInfo(std::string name, float r, float g, float b, float a) {
            name_ = name;
            r_ = r;
            g_ = g;
            b_ = b;
            a_ = a;
        };

        ColorInfo() {};

        std::string getName() {
            return name_;
        };

        float getR() {
            return r_;
        }

        float getG() {
            return g_;
        }

        float getB() {
            return b_;
        }

        float getA() {
            return a_;
        }

    private:
        std::string name_;
        float r_;
        float g_;
        float b_;
        float a_;
};

#endif