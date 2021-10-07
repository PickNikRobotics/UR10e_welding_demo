#include <iostream>
#include <list>

template <typename T>
T GetListElement(std::list<T>& i_list, int i)
{
  if (i < 0 || i >= i_list.size())
  {
    std::cout << "List Size out of range: " << i << std::endl;
    return T();
  }

  typename list<T>::iterator iterNA = i_list.begin();
  std::advance(iterNA, i);
  return *iterNA;
};
