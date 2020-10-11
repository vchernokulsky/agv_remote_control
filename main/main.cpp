#include "Application.h"

extern "C" void app_main();

void app_main() {
    auto *application = new Application();
    application->start();
}
