#include <math.h>
#include <algorithm>
#include "kiss_fftnd.h"
#include "CannyEdgeDetector.h"

#define PI 3.14159265f

CannyEdgeDetector::CannyEdgeDetector()
{
	width = (unsigned int)0;
	height = (unsigned int)0;
	x = (unsigned int)0;
	y = (unsigned int)0;
	mask_halfsize = (unsigned int)0;
}

CannyEdgeDetector::~CannyEdgeDetector()
{
	delete[] edge_magnitude;
	delete[] edge_direction;
	delete[] workspace_bitmap;
}

uint8_t* CannyEdgeDetector::ProcessImage(uint8_t* source_bitmap, unsigned int width,
	unsigned int height, bool useFourier,
	float sigma, uint8_t lowThreshold, uint8_t highThreshold)
{
	this->width = width;
	this->height = height;
	this->source_bitmap = source_bitmap;

	Luminance();
	PreProcessImage(sigma);

	if (useFourier) {
		FourierGaussianBlur(sigma);
		FourierEdgeDetection();
	}
	else {
		GaussianBlur(sigma);
		EdgeDetection();
	}

	NonMaxSuppression();
	Hysteresis(lowThreshold, highThreshold);
	PostProcessImage();

	return this->source_bitmap;
}

inline uint8_t CannyEdgeDetector::GetPixelValue(unsigned int x, unsigned int y)
{
	return (uint8_t) *(workspace_bitmap + (unsigned long)(x * width + y));
}

inline void CannyEdgeDetector::SetPixelValue(unsigned int x, unsigned int y,
	uint8_t value)
{
	workspace_bitmap[(unsigned long)(x * width + y)] = value;
}

void CannyEdgeDetector::PreProcessImage(float sigma)
{
	// Finding mask size with given sigma.
	mask_size = 2 * round(sqrt(-log(0.3) * 2 * sigma * sigma)) + 1;
	mask_halfsize = mask_size / 2;

	// Enlarging workspace bitmap width and height.
	height += mask_halfsize * 2;
	width += mask_halfsize * 2;
	// Working area.
	workspace_bitmap = new uint8_t[height * width];

	// Edge information arrays.
	edge_magnitude = new float[width * height];
	edge_direction = new uint8_t[width * height];

	// Zeroing direction array.
	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			edge_direction[x * width + y] = 0;
		}
	}

	// Copying image data into work area.
	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			// Upper left corner.
			if (x < mask_halfsize &&  y < mask_halfsize) {
				SetPixelValue(x, y, *(source_bitmap));
			}
			// Bottom left corner.
			else if (x >= height - mask_halfsize && y < mask_halfsize) {
				SetPixelValue(x, y, *(source_bitmap + (height - 2 * mask_halfsize - 1) * 3 * (width - 2 * mask_halfsize)));
			}
			// Upper right corner.
			else if (x < mask_halfsize && y >= width - mask_halfsize) {
				SetPixelValue(x, y, *(source_bitmap + 3 * (width - 2 * mask_halfsize - 1)));
			}
			// Bottom right corner.
			else if (x >= height - mask_halfsize && y >= width - mask_halfsize) {
				SetPixelValue(x, y, *(source_bitmap +
					(height - 2 * mask_halfsize - 1) * 3 * (width - 2 * mask_halfsize) + 3 * (width - 2 * mask_halfsize - 1)));
			}
			// Upper beam.
			else if (x < mask_halfsize) {
				SetPixelValue(x, y, *(source_bitmap + 3 * (y - mask_halfsize)));
			}
			// Bottom beam.
			else if (x >= height - mask_halfsize) {
				SetPixelValue(x, y, *(source_bitmap +
					(height - 2 * mask_halfsize - 1) * 3 * (width - 2 * mask_halfsize) + 3 * (y - mask_halfsize)));
			}
			// Left beam.
			else if (y < mask_halfsize) {
				SetPixelValue(x, y, *(source_bitmap +
					(x - mask_halfsize) * 3 * (width - 2 * mask_halfsize)));
			}
			// Right beam.
			else if (y >= width - mask_halfsize) {
				SetPixelValue(x, y, *(source_bitmap +
					(x - mask_halfsize) * 3 * (width - 2 * mask_halfsize) + 3 * (width - 2 * mask_halfsize - 1)));
			}
			// The rest of the image.
			else {
				SetPixelValue(x, y, *(source_bitmap +
					(x - mask_halfsize) * 3 * (width - 2 * mask_halfsize) + 3 * (y - mask_halfsize)));
			}
		}
	}
}

void CannyEdgeDetector::PostProcessImage()
{
	// Decreasing width and height.
	unsigned long i;
	height -= 2 * mask_halfsize;
	width -= 2 * mask_halfsize;

	// Shrinking image.
	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			i = (unsigned long)(x * 3 * width + 3 * y);
			*(source_bitmap + i) =
				*(source_bitmap + i + 1) =
				*(source_bitmap + i + 2) = workspace_bitmap[(x + mask_halfsize) * (width + 2 * mask_halfsize) + (y + mask_halfsize)];
		}
	}
}

void CannyEdgeDetector::Luminance()
{
	unsigned long i;
	float gray_value, blue_value, green_value, red_value;

	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {

			// Current "B" pixel position in bitmap table (calculated with x and y values).
			i = (unsigned long)(x * 3 * width + 3 * y);

			// The order of bytes is BGR.
			blue_value = *(source_bitmap + i);
			green_value = *(source_bitmap + i + 1);
			red_value = *(source_bitmap + i + 2);

			// Standard equation from RGB to grayscale.
			gray_value = (uint8_t)(0.299f * red_value + 0.587f * green_value + 0.114f * blue_value);

			// Ultimately making picture grayscale.
			*(source_bitmap + i) =
				*(source_bitmap + i + 1) =
				*(source_bitmap + i + 2) = gray_value;
		}
	}
}

void CannyEdgeDetector::GaussianBlur(float sigma)
{
	// We already calculated mask size in PreProcessImage.
	long signed_mask_halfsize;
	signed_mask_halfsize = this->mask_halfsize;

	float *gaussianMask;
	gaussianMask = new float[mask_size * mask_size];

	for (int i = -signed_mask_halfsize; i <= signed_mask_halfsize; i++) {
		for (int j = -signed_mask_halfsize; j <= signed_mask_halfsize; j++) {
			gaussianMask[(i + signed_mask_halfsize) * mask_size + j + signed_mask_halfsize]
				= (1 / (2 * PI * sigma * sigma)) * exp(-(i * i + j * j) / (2 * sigma * sigma));
		}
	}

	unsigned long i;
	unsigned long i_offset;
	int row_offset;
	int col_offset;
	float new_pixel;

	double initialRMS = 0.0;
	for (int i = 0; i < width * height; ++i)
		initialRMS += workspace_bitmap[i] * workspace_bitmap[i];
	initialRMS = sqrt(initialRMS / width * height);

	for (x = signed_mask_halfsize; x < height - signed_mask_halfsize; x++) {
		for (y = signed_mask_halfsize; y < width - signed_mask_halfsize; y++) {
			new_pixel = 0;
			for (row_offset = -signed_mask_halfsize; row_offset <= signed_mask_halfsize; row_offset++) {
				for (col_offset = -signed_mask_halfsize; col_offset <= signed_mask_halfsize; col_offset++) {
					i_offset = (unsigned long)((x + row_offset) * width + (y + col_offset));
					new_pixel += (float)((workspace_bitmap[i_offset])) * gaussianMask[(signed_mask_halfsize + row_offset) * mask_size + signed_mask_halfsize + col_offset];
				}
			}
			i = (unsigned long)(x * width + y);
			workspace_bitmap[i] = new_pixel;
		}
	}

	double resultRMS = 0.0;
	for (int i = 0; i < width * height; ++i)
		resultRMS += workspace_bitmap[i] * workspace_bitmap[i];
	resultRMS = sqrt(resultRMS / width * height);

	float coeff = initialRMS / resultRMS;
	for (int i = 0; i < width * height; ++i)
		workspace_bitmap[i] *= coeff;

	delete[] gaussianMask;
}

void CannyEdgeDetector::EdgeDetection()
{
	// Sobel masks.
	float Gx[9];
	Gx[0] = 1.0; Gx[1] = 0.0; Gx[2] = -1.0;
	Gx[3] = 2.0; Gx[4] = 0.0; Gx[5] = -2.0;
	Gx[6] = 1.0; Gx[7] = 0.0; Gx[8] = -1.0;
	float Gy[9];
	Gy[0] = -1.0; Gy[1] = -2.0; Gy[2] = -1.0;
	Gy[3] = 0.0; Gy[4] = 0.0; Gy[5] = 0.0;
	Gy[6] = 1.0; Gy[7] = 2.0; Gy[8] = 1.0;

	float value_gx, value_gy;

	float max = 0.0;
	float angle = 0.0;

	// Convolution.
	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			value_gx = 0.0;
			value_gy = 0.0;

			for (int k = 0; k < 3; k++) {
				for (int l = 0; l < 3; l++) {
					value_gx += Gx[l * 3 + k] * GetPixelValue((x + 1) + (1 - k),
						(y + 1) + (1 - l));
					value_gy += Gy[l * 3 + k] * GetPixelValue((x + 1) + (1 - k),
						(y + 1) + (1 - l));
				}
			}

			edge_magnitude[x * width + y] = sqrt(value_gx * value_gx + value_gy * value_gy);// / 4.0;

			// Maximum magnitude.
			max = edge_magnitude[x * width + y] > max ? edge_magnitude[x * width + y] : max;

			// Angle calculation.
			if ((value_gx != 0.0) || (value_gy != 0.0)) {
				angle = atan2(value_gy, value_gx) * 180.0f / PI;
			}
			else {
				angle = 0.0;
			}
			if (((angle > -22.5) && (angle <= 22.5)) ||
				((angle > 157.5) && (angle <= -157.5))) {
				edge_direction[x * width + y] = 0;
			}
			else if (((angle > 22.5) && (angle <= 67.5)) ||
				((angle > -157.5) && (angle <= -112.5))) {
				edge_direction[x * width + y] = 45;
			}
			else if (((angle > 67.5) && (angle <= 112.5)) ||
				((angle > -112.5) && (angle <= -67.5))) {
				edge_direction[x * width + y] = 90;
			}
			else if (((angle > 112.5) && (angle <= 157.5)) ||
				((angle > -67.5) && (angle <= -22.5))) {
				edge_direction[x * width + y] = 135;
			}
		}
	}

	//double RMS = 0.0;
	//for (int i = 0; i < width * height; ++i)
	//	RMS += edge_magnitude[i] * edge_magnitude[i];
	//RMS = sqrt(RMS / width * height);
	//float coeff = 255.0 / RMS;

	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			edge_magnitude[x * width + y] = 255.0f * edge_magnitude[x * width + y] / max;
			//edge_magnitude[x * width + y] *= coeff;
			SetPixelValue(x, y, edge_magnitude[x * width + y]);
		}
	}
}

kiss_fft_cpx complexMult(const kiss_fft_cpx& op1, const kiss_fft_cpx& op2) {
	kiss_fft_cpx res;
	res.r = op1.r * op2.r - op1.i * op2.i;
	res.i = op1.r * op2.i + op1.i * op2.r;
	return res;

}

// Root Mean Square
double getArrayRMS(kiss_fft_cpx* arr, int size) {
	double RMS = 0.0;
	
	for (int i = 0; i < size; ++i)
		RMS += arr[i].r * arr[i].r;
	
	RMS = sqrt(RMS / size);
	return RMS;
}

void multAllArrayElementsByValue(kiss_fft_cpx* arr, int size, float val) {
	for (int i = 0; i < size; ++i)
		arr[i].r *= val;
}

void expandMask(float* kernel, int initialKernelSize, int resultWidth, int resultHeight, float* mem) {
	int small = initialKernelSize;
	int smallHalf = initialKernelSize / 2;
	int bigW = resultWidth;
	int bigH = resultHeight;

	for (int i = 0; i < bigW * bigH; ++i)
		mem[i] = 0.f;

	for (int i = 0; i < small - smallHalf; ++i)
		for (int j = 0; j < small - smallHalf; ++j)
			mem[i * bigW + j] = kernel[(i + smallHalf) * small + j + smallHalf];
	for (int i = 0; i < small - smallHalf; ++i)
		for (int j = bigW - smallHalf; j < bigW; ++j)
			mem[i * bigW + j] = kernel[(i + smallHalf) * small + j - (bigW - smallHalf)];
	for (int i = bigH - smallHalf; i < bigH; ++i)
		for (int j = 0; j < small - smallHalf; ++j)
			mem[i * bigW + j] = kernel[(i - (bigH - smallHalf)) * small + j + smallHalf];
	for (int i = bigH - smallHalf; i < bigH; ++i)
		for (int j = bigW - smallHalf; j < bigW; ++j)
			mem[i * bigW + j] = kernel[(i - (bigH - smallHalf)) * small + (j - (bigW - smallHalf))];
}

kiss_fft_cpx* getFourierMultiplicationResult(float* image, float* mask, int width, int height) {
	const int dim[2] = { height, width }; // dimensions of fft
	const int dimcount = 2; // number of dimensions. here 2
	kiss_fftnd_cfg stf = kiss_fftnd_alloc(dim, dimcount, 0, 0, 0); // forward 2d
	kiss_fftnd_cfg sti = kiss_fftnd_alloc(dim, dimcount, 1, 0, 0); // inverse 2d

	kiss_fft_cpx *inputImage = new kiss_fft_cpx[width * height];
	kiss_fft_cpx *inputMask = new kiss_fft_cpx[width * height];
	kiss_fft_cpx *outputImage = new kiss_fft_cpx[width * height];
	kiss_fft_cpx *outputMask = new kiss_fft_cpx[width * height];

	for (unsigned int i = 0; i < height * width; i++) {
		inputImage[i].r = image[i];
		inputImage[i].i = 0.f;

		inputMask[i].r = mask[i];
		inputMask[i].i = 0.f;
	}

	double inputImageRMS = getArrayRMS(inputImage, width * height);
	kiss_fftnd(stf, inputImage, outputImage);
	kiss_fftnd(stf, inputMask, outputMask);

	// FFT results multiplication
	for (unsigned int i = 0; i < height * width; i++)
		outputImage[i] = complexMult(outputImage[i], outputMask[i]);

	kiss_fftnd(sti, outputImage, inputImage);
	double resultImageRMS = getArrayRMS(inputImage, width * height);

	float coeff = inputImageRMS / resultImageRMS;
	multAllArrayElementsByValue(inputImage, width * height, coeff);

	delete[] inputMask, outputImage, outputMask;

	return inputImage;
}

//Only square images are supported (size is a power of 2)
void CannyEdgeDetector::FourierGaussianBlur(float sigma) {
	// We already calculated mask size in PreProcessImage.
	long signed_mask_halfsize;
	signed_mask_halfsize = this->mask_halfsize;

	float *gaussianMask;
	gaussianMask = new float[mask_size * mask_size];

	for (int i = -signed_mask_halfsize; i <= signed_mask_halfsize; i++) {
		for (int j = -signed_mask_halfsize; j <= signed_mask_halfsize; j++) {
			gaussianMask[(i + signed_mask_halfsize) * mask_size + j + signed_mask_halfsize]
				= (1 / (2 * PI * sigma * sigma)) * exp(-(i * i + j * j) / (2 * sigma * sigma));
		}
	}

	// Mask
	float* expandedMask = new float[width * height];
	expandMask(gaussianMask, mask_size, width, height, expandedMask);

	// Image
	float* expandedImage = new float[width * height];
	int origImageHeight = height - 2 * mask_halfsize;
	int origImageWidth = width - 2 * mask_halfsize;
	int heighBound = mask_halfsize;
	int lowBound = heighBound + origImageHeight;
	int leftBound = mask_halfsize;
	int rightBound = leftBound + origImageWidth;
	int i = 0;

	for (i = 0; i < heighBound; ++i)
		for (int j = 0; j < width; ++j)
			expandedImage[i * width + j] = 0.f;
	for (; i < lowBound; ++i)
		for (int j = 0; j < width; ++j)
			if (j < leftBound)
				expandedImage[i * width + j] = 0.f;
			else if (j < rightBound)
				expandedImage[i * width + j] = (source_bitmap[(i - heighBound) * origImageWidth * 3 + (j - leftBound) * 3]);
			else
				expandedImage[i * width + j] = 0.f;
	for (; i < height; ++i)
		for (int j = 0; j < width; ++j)
			expandedImage[i * width + j] = 0.f;

	// FFT
	kiss_fft_cpx* result = getFourierMultiplicationResult(expandedImage, expandedMask, width, height);
	for (int i = 0; i < width; ++i)
		for (int j = 0; j < height; ++j)
			workspace_bitmap[i * width + j] = result[i * width + j].r;
}

//Only square images are supported (size is a power of 2)
void CannyEdgeDetector::FourierEdgeDetection() {
	// Sobel masks.
	float Gx[9];
	Gx[0] = 1.0; Gx[1] = 0.0; Gx[2] = -1.0;
	Gx[3] = 2.0; Gx[4] = 0.0; Gx[5] = -2.0;
	Gx[6] = 1.0; Gx[7] = 0.0; Gx[8] = -1.0;
	float Gy[9];
	Gy[0] = -1.0; Gy[1] = -2.0; Gy[2] = -1.0;
	Gy[3] = 0.0; Gy[4] = 0.0; Gy[5] = 0.0;
	Gy[6] = 1.0; Gy[7] = 2.0; Gy[8] = 1.0;

	float* image = new float[width * height];
	for (int i = 0; i < width * height; ++i)
		image[i] = workspace_bitmap[i];
	
	float* expandedGx = new float[width * height];
	expandMask(Gx, 3, width, height, expandedGx);

	float* expandedGy = new float[width * height];
	expandMask(Gy, 3, width, height, expandedGy);

	kiss_fft_cpx* resultGx = getFourierMultiplicationResult(image, expandedGx, width, height);
	kiss_fft_cpx* resultGy = getFourierMultiplicationResult(image, expandedGy, width, height);

	float max = 0.0;
	float angle = 0.0;
	for (x = 0; x < height; x++)
		for (y = 0; y < width; y++) {
			float value_gx = resultGx[x * width + y].r;
			float value_gy = resultGy[x * width + y].r;

			edge_magnitude[x * width + y] = sqrt(value_gx * value_gx + value_gy * value_gy) / 4.f;

			// Maximum magnitude.
			max = edge_magnitude[x * width + y] > max ? edge_magnitude[x * width + y] : max;

			// Angle calculation.
			if ((value_gx != 0.f) || (value_gy != 0.f)) {
				angle = atan2(value_gy, value_gx) * 180.0f / PI;
			}
			else {
				angle = 0.0;
			}
			if (((angle > -22.5) && (angle <= 22.5)) ||
				((angle > 157.5) && (angle <= -157.5))) {
				edge_direction[x * width + y] = 0;
			}
			else if (((angle > 22.5) && (angle <= 67.5)) ||
				((angle > -157.5) && (angle <= -112.5))) {
				edge_direction[x * width + y] = 45;
			}
			else if (((angle > 67.5) && (angle <= 112.5)) ||
				((angle > -112.5) && (angle <= -67.5))) {
				edge_direction[x * width + y] = 90;
			}
			else if (((angle > 112.5) && (angle <= 157.5)) ||
				((angle > -67.5) && (angle <= -22.5))) {
				edge_direction[x * width + y] = 135;
			}
		}

	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			edge_magnitude[x * width + y] = 255.f * edge_magnitude[x * width + y] / max;
			SetPixelValue(x, y, edge_magnitude[x * width + y]);
		}
	}
}

void CannyEdgeDetector::NonMaxSuppression()
{
	float pixel_1 = 0;
	float pixel_2 = 0;
	float pixel;

	for (x = 1; x < height - 1; x++) {
		for (y = 1; y < width - 1; y++) {
			if (edge_direction[x * width + y] == 0) {
				pixel_1 = edge_magnitude[(x + 1) * width + y];
				pixel_2 = edge_magnitude[(x - 1) * width + y];
			}
			else if (edge_direction[x * width + y] == 45) {
				pixel_1 = edge_magnitude[(x + 1) * width + y - 1];
				pixel_2 = edge_magnitude[(x - 1) * width + y + 1];
			}
			else if (edge_direction[x * width + y] == 90) {
				pixel_1 = edge_magnitude[x * width + y - 1];
				pixel_2 = edge_magnitude[x * width + y + 1];
			}
			else if (edge_direction[x * width + y] == 135) {
				pixel_1 = edge_magnitude[(x + 1) * width + y + 1];
				pixel_2 = edge_magnitude[(x - 1) * width + y - 1];
			}
			pixel = edge_magnitude[x * width + y];
			if ((pixel >= pixel_1) && (pixel >= pixel_2)) {
				SetPixelValue(x, y, pixel);
			}
			else {
				SetPixelValue(x, y, 0);
			}
		}
	}

	bool change = true;
	while (change) {
		change = false;
		for (x = 1; x < height - 1; x++) {
			for (y = 1; y < width - 1; y++) {
				if (GetPixelValue(x, y) == 255) {
					if (GetPixelValue(x + 1, y) == 128) {
						change = true;
						SetPixelValue(x + 1, y, 255);
					}
					if (GetPixelValue(x - 1, y) == 128) {
						change = true;
						SetPixelValue(x - 1, y, 255);
					}
					if (GetPixelValue(x, y + 1) == 128) {
						change = true;
						SetPixelValue(x, y + 1, 255);
					}
					if (GetPixelValue(x, y - 1) == 128) {
						change = true;
						SetPixelValue(x, y - 1, 255);
					}
					if (GetPixelValue(x + 1, y + 1) == 128) {
						change = true;
						SetPixelValue(x + 1, y + 1, 255);
					}
					if (GetPixelValue(x - 1, y - 1) == 128) {
						change = true;
						SetPixelValue(x - 1, y - 1, 255);
					}
					if (GetPixelValue(x - 1, y + 1) == 128) {
						change = true;
						SetPixelValue(x - 1, y + 1, 255);
					}
					if (GetPixelValue(x + 1, y - 1) == 128) {
						change = true;
						SetPixelValue(x + 1, y - 1, 255);
					}
				}
			}
		}
		if (change) {
			for (x = height - 2; x > 0; x--) {
				for (y = width - 2; y > 0; y--) {
					if (GetPixelValue(x, y) == 255) {
						if (GetPixelValue(x + 1, y) == 128) {
							change = true;
							SetPixelValue(x + 1, y, 255);
						}
						if (GetPixelValue(x - 1, y) == 128) {
							change = true;
							SetPixelValue(x - 1, y, 255);
						}
						if (GetPixelValue(x, y + 1) == 128) {
							change = true;
							SetPixelValue(x, y + 1, 255);
						}
						if (GetPixelValue(x, y - 1) == 128) {
							change = true;
							SetPixelValue(x, y - 1, 255);
						}
						if (GetPixelValue(x + 1, y + 1) == 128) {
							change = true;
							SetPixelValue(x + 1, y + 1, 255);
						}
						if (GetPixelValue(x - 1, y - 1) == 128) {
							change = true;
							SetPixelValue(x - 1, y - 1, 255);
						}
						if (GetPixelValue(x - 1, y + 1) == 128) {
							change = true;
							SetPixelValue(x - 1, y + 1, 255);
						}
						if (GetPixelValue(x + 1, y - 1) == 128) {
							change = true;
							SetPixelValue(x + 1, y - 1, 255);
						}
					}
				}
			}
		}
	}

	// Suppression
	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			if (GetPixelValue(x, y) == 128) {
				SetPixelValue(x, y, 0);
			}
		}
	}
}

void CannyEdgeDetector::Hysteresis(uint8_t lowThreshold, uint8_t highThreshold)
{
	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			if (GetPixelValue(x, y) >= highThreshold) {
				SetPixelValue(x, y, 255);
				this->HysteresisRecursion(x, y, lowThreshold);
			}
		}
	}

	for (x = 0; x < height; x++) {
		for (y = 0; y < width; y++) {
			if (GetPixelValue(x, y) != 255) {
				SetPixelValue(x, y, 0);
			}
		}
	}
}

void CannyEdgeDetector::HysteresisRecursion(long x, long y, uint8_t lowThreshold)
{
	uint8_t value = 0;

	for (long x1 = x - 1; x1 <= x + 1; x1++) {
		for (long y1 = y - 1; y1 <= y + 1; y1++) {
			if ((x1 < height) & (y1 < width) & (x1 >= 0) & (y1 >= 0)
				& (x1 != x) & (y1 != y)) {

				value = GetPixelValue(x1, y1);
				if (value != 255) {
					if (value >= lowThreshold) {
						SetPixelValue(x1, y1, 255);
						this->HysteresisRecursion(x1, y1, lowThreshold);
					}
					else {
						SetPixelValue(x1, y1, 0);
					}
				}
			}
		}
	}
}
