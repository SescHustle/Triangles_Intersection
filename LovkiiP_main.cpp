#include "LovkiiP_triangles.h"
int main(){
    std::array<double, 9> data1;
    std::array<double, 9> data2;

    for (int i = 0; i!=9; ++i){
        std::cin >> data1[i];
    }

    for (int i = 0; i!=9; ++i){
        std::cin >> data2[i];
    }

    std::cout << LovkiiP::intersection(data1, data2);
}

