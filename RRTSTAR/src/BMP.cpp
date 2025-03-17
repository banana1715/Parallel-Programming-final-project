#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "BMP.h"

Graph::Graph(int width, int height) : width(width), height(height), image_path("test.bmp")
{
    this->image = std::vector<uint8_t>((width) * (height) * 3, 255);
}

Graph::Graph(int width, int height, std::string image_path) : width(width), height(height)
{
    if (image_path == "")
        this->image_path = "test.bmp";
    else
        this->image_path = image_path;
    this->image = std::vector<uint8_t>((width) * (height) * 3, 255);
}

Graph::~Graph() {}

int Graph::getIndex(int x, int y)
{
    return ((this->height - 1 - y) * this->width + x) * 3; 
}

void Graph::DrawLine(int x1, int y1, int x2, int y2, Color color)
{
    int dx = abs(x2 - x1), dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        // Set the pixel
        if (x1 >= 0 && x1 < this->width && y1 >= 0 && y1 < this->height) {
            this->image[this->getIndex(x1, y1) + 0] = color.Red;
            this->image[this->getIndex(x1, y1) + 1] = color.Green;
            this->image[this->getIndex(x1, y1) + 2] = color.Blue;
        }

        // Break if the end point is reached
        if (x1 == x2 && y1 == y2) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

void Graph::FillArea(int x1, int y1, int x2, int y2, Color color)
{
    int left = std::min(x1, x2);
    int right = std::max(x1, x2);
    int bottom = std::min(y1, y2);
    int top = std::max(y1, y2);

    // Iterate over the area and set the values
    for (int y = bottom; y <= top; ++y) {
        for (int x = left; x <= right; ++x) {
            if (x >= 0 && x < this->width && y >= 0 && y < this->height) { // Boundary check
                image[this->getIndex(x, y) + 0] = color.Red;
                image[this->getIndex(x, y) + 1] = color.Green;
                image[this->getIndex(x, y) + 2] = color.Blue;
            }
        }
    }
}

void Graph::DrawRect(int x1, int y1, int x2, int y2, Color color, bool fill, Color Areacolor) {
    int left = std::min(x1, x2);
    int right = std::max(x1, x2);
    int bottom = std::min(y1, y2);
    int top = std::max(y1, y2);

    if (fill)
        this->FillArea(x1, y1, x2, y2, Areacolor);

    this->DrawLine(left, bottom, right, bottom, color);
    this->DrawLine(left, bottom, left, top, color);
    this->DrawLine(left, top, right, top, color);
    this->DrawLine(right, bottom, right, top, color);
}

void Graph::DrawRect(int x1, int y1, int x2, int y2, Color color) {
    this->DrawRect(x1, y1, x2, y2, color, false, {0, 0, 0});
}

void Graph::DrawStar(int X, int Y, int pixel, Color color) {
    this->DrawLine(std::max(0, X - pixel), Y, std::min(width-1, X + pixel), Y, color);
    this->DrawLine(X, std::max(0, Y - pixel), X, std::min(height - 1, Y + pixel), color);
    this->DrawLine(std::max(0, X - pixel), std::max(0, Y - pixel), std::min(width-1, X + pixel), std::min(height - 1, Y + pixel), color);
    this->DrawLine(std::max(0, X - pixel), std::min(height - 1, Y + pixel), std::min(width-1, X + pixel), std::max(0, Y - pixel), color);
}

void Graph::SetUpBackGround() {
    this->DrawRect(0, 0, width - 1, height - 1, {0, 0, 0});
}

void Graph::WriteImage() {
    // Create BMP headers
    BMPFileHeader fileHeader = {
        .bfType = 0x4D42, // 'BM' in little-endian
        .bfSize = static_cast<uint32_t>(sizeof(BMPFileHeader) + sizeof(BMPInfoHeader) + this->getImageSize()),
        .bfReserved1 = 0,
        .bfReserved2 = 0,
        .bfOffBits = sizeof(BMPFileHeader) + sizeof(BMPInfoHeader)
    };

    BMPInfoHeader infoHeader = {
        .biSize = sizeof(BMPInfoHeader),
        .biWidth = width,
        .biHeight = -height, // Negative for top-down row order
        .biPlanes = 1,
        .biBitCount = 24,
        .biCompression = 0,
        .biSizeImage = 0,
        .biXPelsPerMeter = 0,
        .biYPelsPerMeter = 0,
        .biClrUsed = 0,
        .biClrImportant = 0
    };


    // Write BMP to file
    FILE *file = fopen(this->image_path.c_str(), "wb");
    if (!file) {
        perror("Failed to open file");
        exit(-1);
    }

    fwrite(&fileHeader, sizeof(fileHeader), 1, file);
    fwrite(&infoHeader, sizeof(infoHeader), 1, file);
    fwrite(this->getImageData(), this->getImageSize(), 1, file);
    // fwrite(image.data(), image.size(), 1, file);

    fclose(file);
}

void Graph::CheckPoint() {
    this->ckpt = this->image;
}

void Graph::Restore() {
    this->image = this->ckpt;
}