#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char** argv) {
  
  cv::Mat image;

  image = cv::imread(argv[1]);

  if (image.data == nullptr) {
    std::cerr << "File " << argv[1] << "not exist !" << std::endl;
    return 0;
  }

  std::cout << "image width = " << image.cols 
            << ", high = " << image.rows 
            << ", channels = " << image.channels() 
            << std::endl;

  cv::imshow("image", image);
  cv::waitKey(0);

  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    std::cout << "please input a color image or gray image !" << std::endl;
    return 0;
  }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  for (size_t y = 0; y < image.rows; y++) {
    for (size_t x = 0; x < image.cols; x++) {
      unsigned char* row_ptr = image.ptr<unsigned char> (y);
      unsigned char* data_ptr = &row_ptr[x * image.channels()];

      for (int c = 0; c != image.channels(); c++) {
        // I(x, y) = data_ptr[c]
        unsigned char data = data_ptr[c];
      }
    }
  }

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = 
            std::chrono::duration_cast<std::chrono::duration<double>> (t2 - t1);

  std::cout << "image time use = " << time_used.count() << "s" << std::endl;
  
  cv::Mat image_another = image;
  image_another(cv::Rect(0, 0, 100, 100)).setTo(0);
  cv::imshow("image", image);
  cv::waitKey(0);

  cv::Mat image_clone = image.clone();
  image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image", image);
  cv::imshow("image_chone", image_clone);
  cv::waitKey(0);
  
  cv::destroyAllWindows();

  return 0;
}
