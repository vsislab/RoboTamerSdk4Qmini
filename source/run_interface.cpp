#include "user/custom.hpp"

int main(int argc, char const *argv[]) {
    std::cout << "Usage networkInterface: " << "eth0 of Q1 robot " << std::endl;
    std::string networkInterface = "eth0";
    G1 g1(networkInterface, false);

    while (true) sleep(10);

    return 0;
}
