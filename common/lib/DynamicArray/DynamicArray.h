#ifndef DYNAMIC_ARRAY_H
#define DYNAMIC_ARRAY_H 

template <typename Type>
class DynamicArray {
private:
  Type* m_data;
  size_t m_size;

public:
  DynamicArray() : 
    m_data(nullptr), 
    m_size(0) {

  }

  ~DynamicArray() {
    delete [] m_data;
  }

  void append(Type newData) {
    int newDataSize = m_size + 1;
    Type* dataTemp = new Type[newDataSize];

    for (size_t i = 0; i < m_size; ++i) {
      dataTemp[i] = m_data[i];
    }
    
    dataTemp[newDataSize - 1] = newData;
    m_size++;
    
    delete [] m_data;
    m_data = dataTemp;
  }

  size_t size() {
    return m_size;
  }

  Type at(int i) {
    return m_data[i];
  }

};

#endif