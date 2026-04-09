#include <opencv2/highgui/highgui_c.h>
#include <opencv2/videoio/videoio_c.h>
#include <stdio.h>

int main() {
    // Öppna kameraströmmen (0 = första kameran)
    // CV_CAP_V4L2 tvingar användning av Video4Linux2 som lirar med libcamera
    CvCapture* capture = cvCreateCameraCapture(0);
    
    if (!capture) {
        fprintf(stderr, "FEL: Kunde inte hitta kameran!\n");
        return -1;
    }

    // Sätt upplösningen för bättre prestanda på en RPi 3B+
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);

    // Skapa fönstret
    cvNamedWindow("Kamera-fönster", CV_WINDOW_AUTOSIZE);

    IplImage* frame;

    printf("Visar kamera. Tryck på ESC för att avsluta.\n");

    while (1) {
        // Hämta en bildruta
        frame = cvQueryFrame(capture);
        if (!frame) {
            break;
        }

        // Visa bildrutan
        cvShowImage("Kamera-fönster", frame);

        // Vänta 10ms, avsluta om användaren trycker på ESC (ASCII 27)
        char c = cvWaitKey(10);
        if (c == 27) {
            break;
        }
    }

    // Städa upp
    cvReleaseCapture(&capture);
    cvDestroyWindow("Kamera-fönster");

    return 0;
}