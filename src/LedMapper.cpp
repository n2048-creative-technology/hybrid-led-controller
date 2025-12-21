// LedMapper.cpp
// Copied from cylinder-led-controller (unmodified)

#include "LedMapper.h"

LedMapper::LedMapper() {}

void LedMapper::setRotation(int r) {
    rotation = ofClamp(r, 0, kStripes - 1);
}

void LedMapper::setVerticalFlip(bool flip) {
    verticalFlip = flip;
}

void LedMapper::setSerpentine(bool serp) {
    serpentine = serp;
}

int LedMapper::map(int stripe, int y) const {
    // Apply rotation around cylinder
    int s = (stripe + rotation) % kStripes;
    int Y = verticalFlip ? (kHeight - 1 - y) : y;
    if (serpentine) {
        // Alternate direction per stripe
        if (s % 2 == 1) {
            Y = kHeight - 1 - Y;
        }
    }
    return s * kHeight + Y;
}

void LedMapper::unmap(int index, int &stripe, int &y) const {
    int s = index / kHeight;
    int Y = index % kHeight;
    // Reverse serpentine
    if (serpentine && (s % 2 == 1)) {
        Y = kHeight - 1 - Y;
    }
    // Reverse vertical flip
    int origY = verticalFlip ? (kHeight - 1 - Y) : Y;
    // Reverse rotation
    int origS = (s - rotation) % kStripes;
    if (origS < 0) origS += kStripes;

    stripe = origS;
    y = origY;
}

