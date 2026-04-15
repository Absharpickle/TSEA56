#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0, CAP_V4L2);

    if (!cap.isOpened()) {
        cerr << "FEL: Kunde inte öppna kameran!" << endl;
        return -1;
    }

    // Tvinga kameran till YUYV formatet som vi vet fungerar
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    Mat frame;
    namedWindow("Kamera", WINDOW_AUTOSIZE);

    while (true) {
        if (!cap.read(frame)) {
            cout << "Väntar på bildruta (YUYV)..." << endl;
            continue;
        }

        imshow("Kamera", frame);

        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
