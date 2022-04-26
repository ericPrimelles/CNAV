#include "iostream"
#include <vector>


int main(int argc, char const *argv[])
{
    std::vector<int> v;

    v.reserve(10);

    std::cout << v.size() << std::flush << std::endl;
    return 0;
}
