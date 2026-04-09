#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Öppna kameran med V4L2
    VideoCapture cap(0, CAP_V4L2);

    if (!cap.isOpened()) {
        cerr << "FEL: Kunde inte öppna enheten /dev/video0" << endl;
        return -1;
    }

    // Tvinga fram MJPG och en standardupplösning
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30);

    Mat frame;
    namedWindow("Kamera", WINDOW_AUTOSIZE);

    cout << "Försöker hämta bild..." << endl;

    while (true) {
        if (!cap.read(frame)) {
            // Om den misslyckas, vänta lite och försök igen
            cout << "Väntar på bildruta..." << endl;
            waitKey(500); 
            continue;
        }

        imshow("Kamera", frame);

        if (waitKey(1) == 'q' || waitKey(1) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}