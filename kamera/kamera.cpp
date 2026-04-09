#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Öppna kameran med V4L2-backend (Viktigt för RPi)
    VideoCapture cap(0, CAP_V4L2);

    if (!cap.isOpened()) {
        cerr << "FEL: Kunde inte ansluta till kameran!" << endl;
        return -1;
    }

    // Sätt upplösningen lågt för att spara CPU på din Pi 3B+
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    Mat frame;
    namedWindow("Live-feed", WINDOW_AUTOSIZE);

    cout << "Kameran körs. Tryck på valfri tangent för att avsluta." << endl;

    while (true) {
        cap >> frame; // Hämta ny bildruta

        if (frame.empty()) break;

        imshow("Live-feed", frame); // Visa bilden

        // Vänta 1ms på tangenttryck
        if (waitKey(1) >= 0) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}