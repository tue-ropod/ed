#ifndef ED_ARBITRARY_DATA_BUFFER_H_
#define ED_ARBITRARY_DATA_BUFFER_H_


#include <string>
#include <ed/bounded_buffer.h>

class ArbitrayDataBuffer
{     
        
public:
  ArbitrayDataBuffer( int bufferSizeIn = 100 ): bufferSize(bufferSizeIn){ };
   
  std::string name;
  
  std::string classType;
  
  int bufferSize; // TODO? make configurable
};

#endif
