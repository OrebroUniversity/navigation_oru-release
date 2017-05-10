#pragma once

#include <boost/random.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <vector>
#include <fstream>

namespace orunav_generic
{

  inline int getSeed()
  {
    std::ifstream rand("/dev/urandom");
    char tmp[sizeof(int)];
    rand.read(tmp,sizeof(int));
    rand.close();
    int* number = reinterpret_cast<int*>(tmp);
    return (*number);
  }

  //! Don't use this if you have a very large sample space - should be rather dense.
  class UniformIntWithoutRep
  {
  public:
    UniformIntWithoutRep(int min, int max, unsigned int seed = 0)
      {
	unsigned int size = static_cast<unsigned int>(max - min);
	buf.resize(size);
	assert(min < max);
	for (unsigned int i = 0; i < size; i++)
	  {
	    buf[i] = min + i;
	  }
	if (seed != 0)
	  this->setSeed(seed);
      }
    int drawSample()
    {
      boost::uniform_int<> dist(0, buf.size()-1);
      boost::variate_generator<boost::minstd_rand&, boost::uniform_int<> > dice(rng, dist);
      
      unsigned int rand_val = dice();
      unsigned int rand = buf[rand_val];
  
      buf[rand_val] = buf.back();
      buf.pop_back();

      return rand;
    }

    void setSeed(unsigned int seed)
    {
      rng.seed(seed);
    }
    
  private:
    boost::minstd_rand rng;
    //boost::mt19937 rng;
    std::vector<int> buf;
  };
}
