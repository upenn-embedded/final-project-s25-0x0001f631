#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

uint32_t speed;
int fd;
uint8_t bits;

uint8_t send_spi(uint8_t byte1, uint8_t byte2) {
    uint8_t tx[] = {0xFF, byte1, byte2};
    uint8_t rx[sizeof(tx)] = {0};

struct spi_ioc_transfer tr;
memset(&tr, 0, sizeof(tr));
tr.tx_buf = (unsigned long)tx;
tr.rx_buf = (unsigned long)rx;
tr.len = sizeof(tx);
tr.delay_usecs = 0;
tr.speed_hz = speed;
tr.bits_per_word = bits;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("SPI transfer failed");
        return 1;
    }
    std::cout << "Transmitted: ";
for (size_t i = 0; i < sizeof(tx); i++) {
	printf("0x%02X ", tx[i]);
}
std::cout << std::endl;

    std::cout << "Received: ";
    for (size_t i = 0; i < sizeof(rx); i++) {
        printf("0x%02X ", rx[i]);
    }
    std::cout << std::endl;

    return rx[0];
}


// function to compute the distance between two colors in RGB space
double colorDistance(const cv::Vec3b& color1, const cv::Vec3b& color2) {
    int diffR = color1[2] - color2[2]; // Red channel difference
    int diffG = color1[1] - color2[1]; // Green channel difference
    int diffB = color1[0] - color2[0]; // Blue channel difference
    return std::sqrt(diffR * diffR + diffG * diffG + diffB * diffB);
}

uint8_t looping = 0;

int main() {
//setup spi
const char* device = "/dev/spidev0.0";
uint8_t mode = SPI_MODE_0;
bits = 8;
speed = 10000; // 10 kHz
fd = open(device, O_RDWR);
if (fd < 0) {
    perror("Failed to open SPI device");
    return 1;
}

if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
    perror("Can't set SPI mode");
    return 1;
}

if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
    perror("Can't set bits per word");
    return 1;
}

if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
    perror("Can't set max speed");
    return 1;
}

while (1) {

if (looping) {
    system("libcamera-jpeg --width 640 --height 480 -o /home/kevin/hotwheels/image.jpg"); 

    cv::Mat img = cv::imread("/home/kevin/hotwheels/image.jpg", cv::IMREAD_COLOR);

    if (img.empty()) {
        std::cerr << "Error loading image!" << std::endl;
        return -1;
    }

    cv::Vec3b targetColor(220, 60, 0); // BGR: (Blue, Green, Red)

    double threshold = 125;
	double minDist = 10000;
    int leftmost = img.cols, rightmost = -1;
    int topmost = img.rows, bottommost = -1;

    for (int row = 0; row < img.rows; row++) {
        for (int col = 0; col < img.cols; col++) {
            cv::Vec3b color = img.at<cv::Vec3b>(row, col);

            double distance = colorDistance(color, targetColor);

            //if (distance < threshold) {
            //    cv::circle(img, cv::Point(col, row), 3, cv::Scalar(0, 255, 0), -1); // green circle
            //}
	    if (distance < minDist) {
	    	minDist = distance;
	    }
	    if (distance < threshold && color[0] > color[1] * 2 && color[0] > color[2] * 2) {
	    leftmost = std::min(leftmost, col);
                rightmost = std::max(rightmost, col);
                topmost = std::min(topmost, row);
                bottommost = std::max(bottommost, row);
	    }
        }
    }
//    std::cout << "Left, Top: (" << leftmost << ", " << topmost << ")\n";
//   std::cout << "Right, Bottom: (" << rightmost << ", " << bottommost << ")\n";

    cv::rectangle(img, cv::Point(leftmost, topmost), cv::Point(rightmost, bottommost), cv::Scalar(0, 255, 0), 2); // blue rectangle
										  //
//    std::cout << "Min Distance: " << minDist << std::endl;
	cv::imwrite("/home/keving/hotwheels/labeled_image.jpg", img);

    cv::imshow("Labeled", img);

    cv::waitKey(2);

    uint8_t middle = static_cast<uint8_t>((int)(((leftmost + rightmost)/2.0f)* 255.f/640.f));
    uint8_t height = static_cast<uint8_t>((int)((bottommost - topmost) * 255.f/480.f));
    looping = send_spi(middle, height);
} else {

looping = send_spi(0xFF, 0xFF);

sleep(1);
}
}
close(fd);
    return 0;
}
