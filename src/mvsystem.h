#ifndef __mvsystem_h__
#define __mvsystem_h__

#include <smpl/smpl.h>
#include "automaton.h"

#define SIDEWALK      (0)
#define GOOD_DIR      (1)
#define LANE_CHANGE   (2)
#define BOLD_LEFT     (3)

template<size_t N>
class mvregion_c : public region_c<N>{
  public:
    label_c label;
    mvregion_c() : region_c<N>(){}
    mvregion_c(const float* cin, const float* sin) : region_c<N>(cin, sin){}
};

template<class dynamical_system_tt, class map_tt, class cost_tt, class automaton_product_t>
class mvsystem_c : public system_c<dynamical_system_tt, map_tt, cost_tt>{
  public:
    typedef dynamical_system_tt dynamical_system_t;
    typedef map_tt map_t;
    typedef cost_tt cost_t;

    typedef typename dynamical_system_t::state_t state;
    typedef typename dynamical_system_t::control_t control;
    typedef typename dynamical_system_t::opt_data_t opt_data_t;
    typedef typename dynamical_system_t::trajectory_t trajectory;
    const static size_t N = dynamical_system_t::state_t::N;
    typedef mvregion_c<N> region_t;

    using system_c<dynamical_system_tt, map_tt, cost_tt>::dynamical_system;

    automaton_product_t abar;
    vector<region_t> labeled_regions;

    int insert_rules(vector<automaton_ss_c>& psi){
      for(auto& p : psi)
        abar.insert(p);
      return 0;
    }
    
    label_c get_state_label(const state& s)
    {
      label_c l;
      for(auto& r : labeled_regions){
        if(r.is_inside(s))
          l.insert(r.label);
      }
      return l;
    }
    
    timed_word_c get_timed_word(const label_c& l1, const label_c& l2, float dt)
    {
      label_c lt;

      if(l2[SIDEWALK])
        lt.insert(SIDEWALK);
      
      if(l1[GOOD_DIR] && l2[GOOD_DIR])
        lt.insert(GOOD_DIR);
      else if(l1[GOOD_DIR] || l2[GOOD_DIR])
        lt.insert(LANE_CHANGE);
      
      if(l2[BOLD_LEFT])
        lt.insert(BOLD_LEFT);
      
      timed_word_c tw(dt, lt);
      return tw;
    }

    int get_automaton_cost(const state& si, const state& sf, float dt, cost_t& extend_cost)
    {
      label_c li = get_state_label(si); 
      label_c lf = get_state_label(sf);
      timed_word_c tw = get_timed_word(li, lf, dt);
      extend_cost = abar.get_cost(tw);
      return 0;
    }
    
    int evaluate_extend_cost(const state& si, const state& sf,
        opt_data_t& opt_data, cost_t& extend_cost)
    {
      float total_variation = dynamical_system.evaluate_extend_cost(si, sf, opt_data);
      if(total_variation < 0)
        return 1;
      get_automaton_cost(si, sf, total_variation, extend_cost);
      return 0;
    }
};

#endif
