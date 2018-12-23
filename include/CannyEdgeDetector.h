#ifndef _CANNYEDGEDETECTOR_H_
#define _CANNYEDGEDETECTOR_H_

typedef unsigned char uint8_t;

class CannyEdgeDetector
{
public:

	CannyEdgeDetector();
	~CannyEdgeDetector();

	uint8_t* ProcessImage(uint8_t* source_bitmap, unsigned int width,
		unsigned int height, bool useFourier = false, float sigma = 1.f,
		uint8_t lowThreshold = 15, uint8_t highThreshold = 20);

private:
	uint8_t *source_bitmap;

	uint8_t *workspace_bitmap;

	float *edge_magnitude;

	uint8_t *edge_direction;

	unsigned int width;

	unsigned int height;

	unsigned int x;

	unsigned int y;

	unsigned int mask_size;

	unsigned int mask_halfsize;

	inline uint8_t GetPixelValue(unsigned int x, unsigned int y);

	inline void SetPixelValue(unsigned int x, unsigned int y, uint8_t value);

	void PreProcessImage(float sigma);

	void PostProcessImage();

	void Luminance();

	void GaussianBlur(float sigma);

	void EdgeDetection();

	void FourierGaussianBlur(float sigma);

	void FourierEdgeDetection();

	void NonMaxSuppression();

	void Hysteresis(uint8_t lowThreshold, uint8_t highThreshold);

	void HysteresisRecursion(long x, long y, uint8_t lowThreshold);
};

#endif // #ifndef _CANNYEDGEDETECTOR_H_
