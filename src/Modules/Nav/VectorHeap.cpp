#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

int main() {
  vector<int> my_vector;

  for(int i = 0; i < 10; i++) {
    my_vector.push_back(i);
  }

  vector<int>::iterator it = my_vector.begin();
  int* handle;
  handle = &(*it);

  cout << "Handle: " << *handle << "\n";
  
  make_heap(my_vector.begin(), my_vector.end());

  cout << "Handle: " << *handle << "\n";
  
  return 0;
}
