#include <wirebit/wirebit.hpp>

#include <nonsens/nonsens.hpp>

int main() {
    nonsens::sensor::Gnss gnss;
    gnss.set_pod(nonsens::pod::Gnss{});
    return 0;
}
