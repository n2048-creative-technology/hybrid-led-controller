// LedMapper.h
// Copied from cylinder-led-controller (unmodified)

#pragma once

#include "ofMain.h"

class LedMapper {
public:
    static constexpr int kStripes = 12;
    static constexpr int kHeight = 19;
    static constexpr int kPixelCount = kStripes * kHeight;

    LedMapper();

    void setRotation(int rotation); // 0..11
    void setVerticalFlip(bool flip);
    void setSerpentine(bool serp);

    // Map given stripe and y to linear pixel index 0..227 considering options
    int map(int stripe, int y) const;

    // Map from linear index to stripe and y (inverse)
    void unmap(int index, int &stripe, int &y) const;

    int getRotation() const { return rotation; }
    bool getVerticalFlip() const { return verticalFlip; }
    bool getSerpentine() const { return serpentine; }

private:
    int rotation = 0;
    bool verticalFlip = false;
    bool serpentine = false;
};

