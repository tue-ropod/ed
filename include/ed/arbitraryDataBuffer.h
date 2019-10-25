#ifndef ED_ARBITRARY_DATA_BUFFER_H_
#define ED_ARBITRARY_DATA_BUFFER_H_


#include <string>
#include <ed/bounded_buffer.h>

// https://stackoverflow.com/questions/6026234/c-templated-return-value-with-pure-virtual-function
class ArbitrayDataBuffer
{     
        
public:
  ArbitrayDataBuffer(){};
  
  // virutal function to obtain data? What type for the return?
  
 //  virtual void f() {}; // make BaseClass polymorphic
   
  std::string name;
  
  virtual std::string getType(){ return "ArbitrayDataBuffer"; };
  
  virtual void getData(){};
  
  virtual void bufferData(){};
};

#endif
