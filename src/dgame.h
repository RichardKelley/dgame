#ifndef __dgame_h__
#define __dgame_h__

#include "mvsystem.h"

template<rrts_t>
class dgame_t{
  public:
    typedef rrts_t::system::state state;
    typedef typename rrts_t::system::region_t region_t;

    rrts_t p1, p2;
    
    dgame_t() {}

    void insert_rules(vector<automaton_ss_c>& psi){
      p1.system.insert_rules(psi);
      p2.system.insert_rules(psi);
    };
    
    void insert_regions(region_t& op_region; vector<region_t>& regions){
      p1.system.operating_region = op_region;
      p1.system.insert_regions(regions);
      
      p2.system.operating_region = op_region;
      p2.system.insert_regions(regions);
    }
    
    void initialize(const state& p10, const state& p20){
      p1.initialize(p10);
      p2.initialize(p10);
    }
    
    void iteration(){

      p1.iteration();
      p2.iteration();
    }
};

#endif
