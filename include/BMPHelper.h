#include <fstream>
#include <assert.h>

#define BITMAPFILEHEADER_SIZE 14
#define BITMAPINFOHEADER_SIZE 40
#define BYTES_PER_PIXEL 3 // only 24 bit images are supported

class BMPBase {
protected:
	BMPBase() {};

	static uint32_t getRawDataSize(int32_t w, int32_t h) {
		return w * h * BYTES_PER_PIXEL;
	}

// data structures 'file' and 'info' are used to store an image as BMP file.
// for more details see https://en.wikipedia.org/wiki/BMP_file_format
	using BITMAPFILEHEADER = struct {
		std::uint16_t padding; // not in the BMP standard; used for memory alignment
		std::uint16_t type;
		std::uint32_t size;
		std::uint32_t reserved;
		std::uint32_t offBits;
	};
	BITMAPFILEHEADER file;

	using BITMAPINFOHEADER = struct {
		std::uint32_t size;
		std::int32_t width;
		std::int32_t height;
		std::uint16_t planes;
		std::uint16_t bitCount;
		std::uint32_t compression;
		std::uint32_t sizeImage;
		std::int32_t xPelsPerMeter;
		std::int32_t yPelsPerMeter;
		std::uint32_t clrUsed;
		std::uint32_t clrImportant;
	};
	BITMAPINFOHEADER info;
};

class BMPReader : public BMPBase {
public:
	BMPReader(const char* fpath) : filereader(fpath, std::ifstream::binary) {
		assert(filereader.is_open()); //ERROR: could not open specified file

		filereader.read((char*)&file.type, BITMAPFILEHEADER_SIZE);
		filereader.read((char*)&info, BITMAPINFOHEADER_SIZE);

		assert(info.bitCount == 24); //ERROR: only 24 bit images are supported
	}
	~BMPReader() { filereader.close(); }

	int32_t getWidth() { return info.width; }
	int32_t getHeight() { return info.height; }
	uint32_t getRawDataSize() { return BMPBase::getRawDataSize(getWidth(), getHeight()); }
	void extractRawData(char* ptr) {
		int32_t width = getWidth();
		uint32_t paddingSize = (4 - (width * BYTES_PER_PIXEL % 4)) % 4;
		
		int32_t pixelBytesPerLine = width * BYTES_PER_PIXEL;
		uint32_t totalBytesPerLine = pixelBytesPerLine + paddingSize;

		size_t filePos = BITMAPFILEHEADER_SIZE + BITMAPINFOHEADER_SIZE;
		for (int i = 0; i < getHeight(); ++i) {
			filereader.read(ptr + i * pixelBytesPerLine, pixelBytesPerLine);

			filePos += totalBytesPerLine;
			filereader.seekg(filePos);
		}
	}

private:
	std::ifstream filereader;
};

class BMPWriter : public BMPBase {
public:
	BMPWriter(const char* fpath, int32_t width, int32_t height) :
		filewriter(fpath, std::ofstream::binary),
		w(width),
		h(height)
	{
		//BITMAPFILEHEADER
		file.type = 0x4d42; //same as 'BM' in ASCII
		file.size = calculateFileSize();
		file.reserved = 0;
		file.offBits = 54;

		//BITMAPINFOHEADER
		info.size = 40;
		info.width = w;
		info.height = h;
		info.planes = 1;
		info.bitCount = 24;
		info.compression = 0;
		info.sizeImage = BMPBase::getRawDataSize(w, h);
		info.yPelsPerMeter = 0;
		info.xPelsPerMeter = 0;
		info.clrUsed = 0;
		info.clrImportant = 0;
	}
	~BMPWriter() { filewriter.close(); }

	void write(char* rawData) {
		filewriter.write((char*)&file.type, BITMAPFILEHEADER_SIZE);
		filewriter.write((char*)&info, BITMAPINFOHEADER_SIZE);

		auto bytesPerLine = w * BYTES_PER_PIXEL;
		auto paddingSize = (4 - (w * BYTES_PER_PIXEL % 4)) % 4;
		char* padding = new char[paddingSize];
		for (int i = 0; i < paddingSize; ++i)
			padding[i] = 0;

		for (int i = 0; i < h; ++i) {
			filewriter.write(rawData + i * bytesPerLine, bytesPerLine);
			filewriter.write(padding, paddingSize);
		}
	}

private:
	uint32_t calculateFileSize() {
		uint32_t rawDataSize = BMPBase::getRawDataSize(w, h);
		return rawDataSize + BITMAPFILEHEADER_SIZE + BITMAPINFOHEADER_SIZE;
	}

	std::ofstream filewriter;
	int32_t w;
	int32_t h;
};

