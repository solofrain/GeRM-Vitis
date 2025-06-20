#include "GermaniumDetector.hpp"

int main()
{
    GermaniumDetector det;

    det.network_init();
    det.queue_init();
    det.task_init();

    vTaskStartScheduler();

    while(1);
}
