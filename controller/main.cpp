#include "car.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
//#include <wiringSerial.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace rapidjson;

int main() {
    FILE* fp;
    char readBuffer[100];
    Document d;
    Car car;

//    int fd ;
//    if((fd = serialOpen("/dev/ttyACM0", 9600)) < 0){
//        fprintf(stderr,"Unable to open serial device: %s\n", strerror(errno));
//        return 1;
//    }

    while (1) {
        fp = fopen("../../data/data.json", "r");
        FileReadStream is(fp, readBuffer, sizeof(readBuffer));
        d.ParseStream(is);
        int mode = d["mode"].GetInt();
        char op = d["op"].GetString()[0];
        car.perception.update_src();
        if (mode == 1) {
            car.autopilot();
            printf("auto\n");
            imwrite("../../data/view.jpg", car.perception.res);
        } else {
            printf("%c", op);
//            serialPutchar(fd, op);
//            serialFlush(fd);
            imwrite("../../data/view.jpg", car.perception.src);
        }
        fclose(fp);
    }
    return 0;
}
