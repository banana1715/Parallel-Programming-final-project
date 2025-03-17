#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// Define BMP file header structure
#pragma pack(push, 1)
typedef struct {
    uint16_t bfType;      // Bitmap file type (BM)
    uint32_t bfSize;      // File size in bytes
    uint16_t bfReserved1; // Reserved, must be zero
    uint16_t bfReserved2; // Reserved, must be zero
    uint32_t bfOffBits;   // Offset to pixel data
} BMPFileHeader;

typedef struct {
    uint32_t biSize;          // Header size
    int32_t biWidth;          // Image width
    int32_t biHeight;         // Image height
    uint16_t biPlanes;        // Number of planes (1)
    uint16_t biBitCount;      // Bits per pixel (24 for RGB)
    uint32_t biCompression;   // Compression type (0 = none)
    uint32_t biSizeImage;     // Image size in bytes (can be 0 for uncompressed)
    int32_t biXPelsPerMeter;  // Pixels per meter (horizontal)
    int32_t biYPelsPerMeter;  // Pixels per meter (vertical)
    uint32_t biClrUsed;       // Number of colors used
    uint32_t biClrImportant;  // Important colors
} BMPInfoHeader;
#pragma pack(pop)

typedef struct
{
    uint8_t Red, Green, Blue;
} Color;


class Graph
{
private:
    /* data */
    std::vector<uint8_t> image;
    std::vector<uint8_t> ckpt;
    int width, height;
    std::string image_path;
    int getIndex(int, int);
    void FillArea(int, int, int, int, Color);
public:
    Graph(int width, int height);
    Graph(int width, int height, std::string image_path);
    ~Graph();


    void DrawLine(int x1, int y1, int x2, int y2, Color color);
    void DrawRect(int x1, int y1, int x2, int y2, Color color, bool fill, Color Areacolor);
    void DrawRect(int x1, int y1, int x2, int y2, Color color);
    void DrawStar(int x, int y, int pixel, Color color);
    void* getImageData() { return image.data(); }
    size_t getImageSize() { return image.size(); }

    void SetUpBackGround();
    void WriteImage();
    void CheckPoint();
    void Restore();
};