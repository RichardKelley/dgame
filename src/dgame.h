#ifndef __dgame_h__
#define __dgame_h__

#include "mvsystem.h"
#include <queue>


template<class system_tt> class p1vertex_c;
template<class system_tt> class p2vertex_c;

template<class system_tt>
class p1vertex_c : public vertex_c<system_tt>
{
  public:
    typedef vertex_c<system_tt> vertex_t;
    typedef typename vertex_t::state_t state_t;
    
    p2vertex_c<system_tt>* best_response;

    p1vertex_c() : vertex_t(){}
    p1vertex_c(const state_t& s) : vertex_t(s) {} 
};

template<class system_tt>
class p2vertex_c : public vertex_c<system_tt>
{
  public:
    typedef vertex_c<system_tt> vertex_t;
    typedef typename vertex_t::state_t state_t;
    
    set<p1vertex_c<system_tt>* > best_response_of;

    p2vertex_c() : vertex_t(){}
    p2vertex_c(const state_t& s) : vertex_t(s) {} 
};

template<class system_tt>
class dgame_c{
  public:
    typedef system_tt system_t;
    typedef typename system_t::state state;
    typedef typename system_t::region_t region_t;
    typedef p1vertex_c<system_tt> p1vertex_t;
    typedef p2vertex_c<system_tt> p2vertex_t;
    typedef typename system_t::cost_t cost_t;

#if 0
    struct compare_vertex{
      bool operator()(const vertex_t* pv1, const vertex_t* pv2){
        return pv1->cost_from_root < pv2->cost_from_root;
      }
    };
    priority_queue<vertex_t*, vector<vertex_t*>, compare_vertex> p1q; 
    priority_queue<vertex_t*, vector<vertex_t*>, compare_vertex> p2q; 
#endif

    rrts_c<p1vertex_t, edge_c<system_t> > p1;
    rrts_c<p2vertex_t, edge_c<system_t> > p2;

    dgame_c(bot_lcmgl_t* lcmgl) {
      p1.lcmgl = lcmgl;
      p2.lcmgl = lcmgl;
    }

    void insert_rules(vector< pair<size_t, automaton_ss_c> >& psi){
      p1.system.insert_rules(psi);
      p2.system.insert_rules(psi);
    };
    
    void insert_regions(region_t& op_region, region_t& g1, region_t& g2, 
        vector<region_t>& regions){
      p1.system.operating_region = op_region;
      p1.system.goal_region = g1;
      p1.system.insert_regions(regions);
      
      p2.system.operating_region = op_region;
      p2.system.goal_region = g2;
      p2.system.insert_regions(regions);
    }
    
    void initialize(const state& p10, const state& p20){
      p1.initialize(p10);
      //p1q.push(p1.last_added_vertex);
      
      p2.initialize(p10);
      //p2q.push(p2.last_added_vertex);
    }
   
    bool check_collision_trajectory(p1vertex_t& p1v, p2vertex_t& p2v)
    {
      return false;
    }

    void calculate_best_response(p1vertex_t& v)
    {
      p2vertex_t* best_response = NULL;
      cost_t best_cost;
      for(auto& pv : p2.list_vertices)
      {
        if(check_collision_trajectory(v, *pv))
          continue;
        
        if(pv->cost_from_root < best_cost)
        {
          best_cost = pv->cost_from_root;
          best_response = pv;
        }
      }
      v.best_response = best_response;
      best_response->best_response_of.insert(&v);
    }
    
    void update_best_response_descendents(p1vertex_t& v)
    {
      calculate_best_response(v);
      for(auto& pc : v.children)
        update_best_response_descendents(*(static_cast<p1vertex_t*>(pc)));
    }
    
    void iteration(){
      p1.iteration();
      p1vertex_t* p1l = p1.last_added_vertex;

      p2.iteration();
      p2vertex_t* p2l = p2.last_added_vertex;
      
      if(p2l)
      {
        vector<p2vertex_t*> S1;
        for(auto& pv : p2.list_vertices)
        {
          if(p2l->cost_from_root < pv->cost_from_root)
          {
            S1.push_back(pv);
          }
        }
        for(auto& pv : S1)
        {
          for(auto& pbrv : pv->best_response_of)
            calculate_best_response(*pbrv);
        }
      }
      
      if(p1l)
        update_best_response_descendents(*(static_cast<p1vertex_t*>(p1l)));
    }
};

#endif
