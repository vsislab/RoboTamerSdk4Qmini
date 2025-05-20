#include "user/custom.hpp"

int main(int argc, char const *argv[]) {
    std::cout << "Usage networkInterface: " << "enp5s0 of cyy528 " << std::endl;
    std::string networkInterface = "enp4s0";
    G1 g1(networkInterface, false);

    while (true) sleep(10);

    return 0;
}
