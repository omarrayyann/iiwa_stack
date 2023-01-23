#include <iostream>
#include <conio.h>

using namespace std;

float x = 0;
float y = 0;
float z = 0;
float phi = 0;
float theta = -180;
float arm = -218.4;

float increment = 0.2;

int main()
{
  while (1)
  {
    char c = getch();
    switch (c)
    {
      case 'w':
        y += increment;
        break;
      case 's':
        y -= increment;
        break;
      case 'a':
        x += increment;
        break;
      case 'd':
        x -= increment;
        break;
      default:
        cout << c << endl;
        break;
    }
  }
  return 0;
}
