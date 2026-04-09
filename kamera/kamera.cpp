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
    if (!cap.read(frame)) {
        cout << "Kunde inte läsa frame från kameran - väntar..." << endl;
        continue; // Istället för break, försök igen
    }

    if (frame.empty()) {
        cout << "Tom bildruta!" << endl;
        break;
    }

    imshow("Live-feed", frame);

    if (waitKey(30) >= 0) break; // 30ms ger ca 30 FPS
}

    cap.release();
    destroyAllWindows();
    return 0;
}