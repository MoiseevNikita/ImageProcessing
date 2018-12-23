#include <iostream>
#include "BMPHelper.h"
#include "CannyEdgeDetector.h"

using namespace std;

int main(int argc, char* argv[]) {
	const char* input_image = "input/lena.bmp";
	const char* output_image = "output/lena_Fourier_5.0_6_8.bmp";
	bool use_fourier = true;

	BMPReader bmpReader(input_image);
	int width = bmpReader.getWidth();
	int height = bmpReader.getHeight();

	char* pixels = new char[bmpReader.getRawDataSize()];
	bmpReader.extractRawData(pixels);

	CannyEdgeDetector CED;
	CED.ProcessImage((uint8_t*)pixels, width, height, use_fourier, /*sigma*/5.f, /*lowThreshold*/6, /*highThreshold*/8);

	BMPWriter bmpWriter(output_image, width, height);
	bmpWriter.write(pixels);

	return 0;
}