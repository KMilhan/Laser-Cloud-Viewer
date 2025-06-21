#include "ui.h"

int main(int argc, char **argv) {
    // initialize UI application
    laserApp app;
    app.basicWindow->show(argc, argv);
    app.advancedWindow->show(argc, argv);
    std::cout << "Initialized" << std::endl;

    return Fl::run();
}
