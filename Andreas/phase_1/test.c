#include<stdio.h>


void print_dict(int col){
    int colors[8][3] = {
    {0,0,0},
    {0,0,1},
    {0,1,0},
    {0,1,1},
    {1,0,0},
    {1,0,1},
    {1,1,0},
    {1,1,1}};
    printf("%d, %d, %d\n", colors[col][0], colors[col][1], colors[col][2]);
}

int main(){
    for(int i=0; i <= 8; i++){
        print_dict(i);
    }
    return 0;
}