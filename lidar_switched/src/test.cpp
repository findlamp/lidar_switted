
#include <iostream>
#include <math.h> 
#include <eigen3/Eigen/Dense>
using namespace std;
void generate_transform(Eigen::Matrix4d& tranform, float vector[],float angle){

    float rotation_matrix[3][3] = {{cos(angle),-sin(angle),0},
                                   {sin(angle),cos(angle),0},
                                    {0,0,1}};

    tranform << cos(angle),-sin(angle),0,vector[0],
                sin(angle),cos(angle), 0,vector[1],
                0,0,1,vector[2],
                0,0,0,1;

            
}
void show(float tranform[4][4])
{
    for(int i = 0 ; i < 4 ; i ++){
        for (int j = 0; j <4 ; j++)
          cout << tranform[i][j] <<'\t';  
        cout << endl;
    }
        
        
        
}

float unpackFloat(auto *buf, int i) {
            auto *b = buf;
            uint32_t temp = 0;
            temp = ((b[i+0]) |
                    (b[i+1] << 8) |
                    (b[i+2] << 16) |
                    b[i+3] << 24);
            return *((float *) &temp);
}

int main()
{   
    
    Eigen::Matrix4d left_to_ram;
    Eigen::Matrix4d right_to_ram;
    Eigen::Matrix4d front_to_ram;
    Eigen::Matrix4d ram_to_cog;

    Eigen::Matrix4d left_global;
    Eigen::Matrix4d right_global;
    Eigen::Matrix4d front_global;

    float left_shift[3] = {1.549, 0.267, 0.543};
    float right_shift[3] = {1.549, -0.267, 0.543};
    float front_shift[3] = {0,0,0};
    float cog_shift[3] = {-1.3206, 0.030188, -0.23598};

    generate_transform(left_to_ram,left_shift,2*M_PI/3);
    generate_transform(right_to_ram,right_shift,-2*M_PI/3);
    generate_transform(front_to_ram,front_shift,0.0);
    generate_transform(ram_to_cog,cog_shift,0.0);

    left_global = ram_to_cog * left_to_ram;
    right_global = ram_to_cog * right_to_ram;
    front_global = ram_to_cog * front_to_ram;

    cout << left_global << endl;
    cout << right_global << endl;
    cout << front_global << endl;
    
   /*
   unsigned char data[24] = {21,48,124,66,248,83,186,63,198,211,187,64,0,0,120,60,0,0,174,66,0,0,0,0};
   float x = unpackFloat(&data[0],0);
   float y = unpackFloat(&data[0],4);
   float z = unpackFloat(&data[0],8);
   float i = unpackFloat(&data[0],12);
   cout << x<< " " << y <<" " << z <<" "<<i<<endl; 
    */
   Eigen::VectorXd T = Eigen::VectorXd::Constant(4,1);
   
   left_global.block<4,1>(0,3) = T;
   cout << left_global << endl;



}


